// Benchmark "adder" written by ABC on Thu Jul 18 10:42:39 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n176, new_n177, new_n178, new_n179, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n310, new_n311, new_n312;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  inv040aa1d28x5               g003(.a(\a[5] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[4] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[5] ), .b(\a[6] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n03p5x5 g006(.a(new_n101), .b(new_n99), .c(new_n100), .o1(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[7] ), .b(\a[8] ), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[6] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[5] ), .o1(new_n105));
  orn002aa1n24x5               g010(.a(\a[7] ), .b(\b[6] ), .o(new_n106));
  nand42aa1n04x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  oai112aa1n06x5               g012(.a(new_n106), .b(new_n107), .c(new_n105), .d(new_n104), .o1(new_n108));
  oao003aa1n03x5               g013(.a(\a[8] ), .b(\b[7] ), .c(new_n106), .carry(new_n109));
  oai013aa1n06x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .d(new_n102), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand22aa1n03x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  tech160nm_fiaoi012aa1n04x5   g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  nor022aa1n04x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nand42aa1n03x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor022aa1n08x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand42aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  tech160nm_fioai012aa1n03p5x5 g024(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n120));
  oai012aa1n09x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .o1(new_n121));
  nand42aa1n02x5               g026(.a(new_n100), .b(new_n99), .o1(new_n122));
  nand42aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oai112aa1n03x5               g028(.a(new_n122), .b(new_n123), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  nor043aa1n03x5               g029(.a(new_n108), .b(new_n124), .c(new_n103), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n110), .c(new_n121), .d(new_n125), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n97), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  oaoi03aa1n06x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n129));
  aoi012aa1n12x5               g034(.a(new_n110), .b(new_n121), .c(new_n125), .o1(new_n130));
  nano22aa1n02x4               g035(.a(new_n130), .b(new_n97), .c(new_n126), .out0(new_n131));
  norp02aa1n02x5               g036(.a(new_n131), .b(new_n129), .o1(new_n132));
  nor022aa1n16x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv030aa1n02x5               g038(.a(new_n133), .o1(new_n134));
  nand42aa1n10x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n132), .b(new_n135), .c(new_n134), .out0(\s[11] ));
  oaoi13aa1n02x5               g041(.a(new_n133), .b(new_n135), .c(new_n131), .d(new_n129), .o1(new_n137));
  xnrb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nano23aa1n03x5               g045(.a(new_n133), .b(new_n139), .c(new_n140), .d(new_n135), .out0(new_n141));
  nanp03aa1n02x5               g046(.a(new_n141), .b(new_n97), .c(new_n126), .o1(new_n142));
  inv000aa1d42x5               g047(.a(\a[10] ), .o1(new_n143));
  inv000aa1d42x5               g048(.a(\b[9] ), .o1(new_n144));
  norp02aa1n02x5               g049(.a(\b[8] ), .b(\a[9] ), .o1(new_n145));
  oaoi03aa1n02x5               g050(.a(new_n143), .b(new_n144), .c(new_n145), .o1(new_n146));
  nona23aa1n09x5               g051(.a(new_n140), .b(new_n135), .c(new_n133), .d(new_n139), .out0(new_n147));
  oaoi03aa1n02x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n134), .o1(new_n148));
  oabi12aa1n03x5               g053(.a(new_n148), .b(new_n147), .c(new_n146), .out0(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n130), .c(new_n142), .out0(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n24x5               g056(.a(\a[13] ), .b(\b[12] ), .o(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aobi12aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .out0(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .out0(new_n156));
  nano22aa1n03x7               g061(.a(new_n156), .b(new_n152), .c(new_n153), .out0(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n148), .c(new_n141), .d(new_n129), .o1(new_n158));
  tech160nm_fioaoi03aa1n05x5   g063(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .o1(new_n159));
  inv000aa1n02x5               g064(.a(new_n159), .o1(new_n160));
  nano22aa1n03x5               g065(.a(new_n147), .b(new_n97), .c(new_n126), .out0(new_n161));
  nano22aa1n02x4               g066(.a(new_n130), .b(new_n161), .c(new_n157), .out0(new_n162));
  nano22aa1n02x4               g067(.a(new_n162), .b(new_n158), .c(new_n160), .out0(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  oaoi03aa1n02x5               g069(.a(\a[15] ), .b(\b[14] ), .c(new_n163), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  tech160nm_fixnrc02aa1n04x5   g071(.a(\b[14] ), .b(\a[15] ), .out0(new_n167));
  xnrc02aa1n03x5               g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  nor042aa1n06x5               g073(.a(new_n168), .b(new_n167), .o1(new_n169));
  aoai13aa1n04x5               g074(.a(new_n169), .b(new_n159), .c(new_n149), .d(new_n157), .o1(new_n170));
  oai022aa1n02x5               g075(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n171));
  aob012aa1n02x5               g076(.a(new_n171), .b(\b[15] ), .c(\a[16] ), .out0(new_n172));
  nand23aa1n03x5               g077(.a(new_n161), .b(new_n157), .c(new_n169), .o1(new_n173));
  oai112aa1n06x5               g078(.a(new_n172), .b(new_n170), .c(new_n130), .d(new_n173), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g080(.a(\a[18] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\a[17] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\b[16] ), .o1(new_n178));
  oaoi03aa1n03x5               g083(.a(new_n177), .b(new_n178), .c(new_n174), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[17] ), .c(new_n176), .out0(\s[18] ));
  inv000aa1d42x5               g085(.a(new_n169), .o1(new_n181));
  aoai13aa1n04x5               g086(.a(new_n172), .b(new_n181), .c(new_n158), .d(new_n160), .o1(new_n182));
  nor042aa1n04x5               g087(.a(new_n130), .b(new_n173), .o1(new_n183));
  xroi22aa1d06x4               g088(.a(new_n177), .b(\b[16] ), .c(new_n176), .d(\b[17] ), .out0(new_n184));
  oaih12aa1n02x5               g089(.a(new_n184), .b(new_n183), .c(new_n182), .o1(new_n185));
  oaih22aa1n04x5               g090(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n186));
  oaib12aa1n09x5               g091(.a(new_n186), .b(new_n176), .c(\b[17] ), .out0(new_n187));
  nor002aa1d32x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nand02aa1d10x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nanb02aa1n02x5               g094(.a(new_n188), .b(new_n189), .out0(new_n190));
  inv000aa1d42x5               g095(.a(new_n190), .o1(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n185), .c(new_n187), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n09x5               g098(.a(new_n188), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(new_n178), .b(new_n177), .o1(new_n195));
  oaoi03aa1n02x5               g100(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n196));
  aoai13aa1n02x7               g101(.a(new_n191), .b(new_n196), .c(new_n174), .d(new_n184), .o1(new_n197));
  nor022aa1n16x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nand02aa1d08x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  aoi012aa1n03x5               g105(.a(new_n200), .b(new_n197), .c(new_n194), .o1(new_n201));
  aoi012aa1n03x5               g106(.a(new_n190), .b(new_n185), .c(new_n187), .o1(new_n202));
  nano22aa1n02x4               g107(.a(new_n202), .b(new_n194), .c(new_n200), .out0(new_n203));
  nor002aa1n02x5               g108(.a(new_n201), .b(new_n203), .o1(\s[20] ));
  nano23aa1n06x5               g109(.a(new_n188), .b(new_n198), .c(new_n199), .d(new_n189), .out0(new_n205));
  nand02aa1n03x5               g110(.a(new_n184), .b(new_n205), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  oai012aa1n06x5               g112(.a(new_n207), .b(new_n183), .c(new_n182), .o1(new_n208));
  nona23aa1n09x5               g113(.a(new_n199), .b(new_n189), .c(new_n188), .d(new_n198), .out0(new_n209));
  oaoi03aa1n12x5               g114(.a(\a[20] ), .b(\b[19] ), .c(new_n194), .o1(new_n210));
  inv040aa1n02x5               g115(.a(new_n210), .o1(new_n211));
  oai012aa1d24x5               g116(.a(new_n211), .b(new_n209), .c(new_n187), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[21] ), .b(\b[20] ), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n208), .c(new_n213), .out0(\s[21] ));
  orn002aa1n24x5               g120(.a(\a[21] ), .b(\b[20] ), .o(new_n216));
  aoai13aa1n03x5               g121(.a(new_n214), .b(new_n212), .c(new_n174), .d(new_n207), .o1(new_n217));
  xnrc02aa1n12x5               g122(.a(\b[21] ), .b(\a[22] ), .out0(new_n218));
  aoi012aa1n03x5               g123(.a(new_n218), .b(new_n217), .c(new_n216), .o1(new_n219));
  aobi12aa1n06x5               g124(.a(new_n214), .b(new_n208), .c(new_n213), .out0(new_n220));
  nano22aa1n02x4               g125(.a(new_n220), .b(new_n216), .c(new_n218), .out0(new_n221));
  nor002aa1n02x5               g126(.a(new_n219), .b(new_n221), .o1(\s[22] ));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nano22aa1n12x5               g128(.a(new_n218), .b(new_n216), .c(new_n223), .out0(new_n224));
  and003aa1n02x5               g129(.a(new_n184), .b(new_n224), .c(new_n205), .o(new_n225));
  oaih12aa1n02x5               g130(.a(new_n225), .b(new_n183), .c(new_n182), .o1(new_n226));
  oaoi03aa1n12x5               g131(.a(\a[22] ), .b(\b[21] ), .c(new_n216), .o1(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n212), .c(new_n224), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[22] ), .b(\a[23] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n226), .c(new_n228), .out0(\s[23] ));
  orn002aa1n24x5               g136(.a(\a[23] ), .b(\b[22] ), .o(new_n232));
  inv030aa1n02x5               g137(.a(new_n228), .o1(new_n233));
  aoai13aa1n02x7               g138(.a(new_n230), .b(new_n233), .c(new_n174), .d(new_n225), .o1(new_n234));
  tech160nm_fixnrc02aa1n05x5   g139(.a(\b[23] ), .b(\a[24] ), .out0(new_n235));
  aoi012aa1n03x5               g140(.a(new_n235), .b(new_n234), .c(new_n232), .o1(new_n236));
  aoi012aa1n02x7               g141(.a(new_n229), .b(new_n226), .c(new_n228), .o1(new_n237));
  nano22aa1n02x4               g142(.a(new_n237), .b(new_n232), .c(new_n235), .out0(new_n238));
  nor002aa1n02x5               g143(.a(new_n236), .b(new_n238), .o1(\s[24] ));
  aoai13aa1n06x5               g144(.a(new_n224), .b(new_n210), .c(new_n205), .d(new_n196), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n227), .o1(new_n241));
  nor042aa1n02x5               g146(.a(new_n235), .b(new_n229), .o1(new_n242));
  inv000aa1n03x5               g147(.a(new_n242), .o1(new_n243));
  oaoi03aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .c(new_n232), .o1(new_n244));
  inv000aa1n03x5               g149(.a(new_n244), .o1(new_n245));
  aoai13aa1n12x5               g150(.a(new_n245), .b(new_n243), .c(new_n240), .d(new_n241), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  nano22aa1n02x4               g152(.a(new_n206), .b(new_n224), .c(new_n242), .out0(new_n248));
  oaih12aa1n02x5               g153(.a(new_n248), .b(new_n183), .c(new_n182), .o1(new_n249));
  xnrc02aa1n12x5               g154(.a(\b[24] ), .b(\a[25] ), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  xnbna2aa1n03x5               g156(.a(new_n251), .b(new_n249), .c(new_n247), .out0(\s[25] ));
  nor042aa1n03x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  aoai13aa1n02x7               g159(.a(new_n251), .b(new_n246), .c(new_n174), .d(new_n248), .o1(new_n255));
  xnrc02aa1n02x5               g160(.a(\b[25] ), .b(\a[26] ), .out0(new_n256));
  aoi012aa1n03x5               g161(.a(new_n256), .b(new_n255), .c(new_n254), .o1(new_n257));
  tech160nm_fiaoi012aa1n02p5x5 g162(.a(new_n250), .b(new_n249), .c(new_n247), .o1(new_n258));
  nano22aa1n02x4               g163(.a(new_n258), .b(new_n254), .c(new_n256), .out0(new_n259));
  nor002aa1n02x5               g164(.a(new_n257), .b(new_n259), .o1(\s[26] ));
  nor042aa1n04x5               g165(.a(new_n256), .b(new_n250), .o1(new_n261));
  nano32aa1n03x7               g166(.a(new_n206), .b(new_n261), .c(new_n224), .d(new_n242), .out0(new_n262));
  oai012aa1n06x5               g167(.a(new_n262), .b(new_n183), .c(new_n182), .o1(new_n263));
  oao003aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n264));
  aobi12aa1n12x5               g169(.a(new_n264), .b(new_n246), .c(new_n261), .out0(new_n265));
  xorc02aa1n12x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n263), .c(new_n265), .out0(\s[27] ));
  norp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  inv040aa1n03x5               g173(.a(new_n268), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n242), .b(new_n227), .c(new_n212), .d(new_n224), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n261), .o1(new_n271));
  aoai13aa1n04x5               g176(.a(new_n264), .b(new_n271), .c(new_n270), .d(new_n245), .o1(new_n272));
  aoai13aa1n02x7               g177(.a(new_n266), .b(new_n272), .c(new_n174), .d(new_n262), .o1(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  aoi012aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n269), .o1(new_n275));
  aobi12aa1n06x5               g180(.a(new_n266), .b(new_n263), .c(new_n265), .out0(new_n276));
  nano22aa1n03x7               g181(.a(new_n276), .b(new_n269), .c(new_n274), .out0(new_n277));
  nor002aa1n02x5               g182(.a(new_n275), .b(new_n277), .o1(\s[28] ));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  norb02aa1n02x5               g184(.a(new_n266), .b(new_n274), .out0(new_n280));
  aoai13aa1n02x7               g185(.a(new_n280), .b(new_n272), .c(new_n174), .d(new_n262), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n269), .carry(new_n282));
  aoi012aa1n03x5               g187(.a(new_n279), .b(new_n281), .c(new_n282), .o1(new_n283));
  aobi12aa1n06x5               g188(.a(new_n280), .b(new_n263), .c(new_n265), .out0(new_n284));
  nano22aa1n03x7               g189(.a(new_n284), .b(new_n279), .c(new_n282), .out0(new_n285));
  norp02aa1n03x5               g190(.a(new_n283), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  norb03aa1n02x5               g193(.a(new_n266), .b(new_n279), .c(new_n274), .out0(new_n289));
  aoai13aa1n02x7               g194(.a(new_n289), .b(new_n272), .c(new_n174), .d(new_n262), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n291));
  aoi012aa1n02x5               g196(.a(new_n288), .b(new_n290), .c(new_n291), .o1(new_n292));
  aobi12aa1n06x5               g197(.a(new_n289), .b(new_n263), .c(new_n265), .out0(new_n293));
  nano22aa1n03x7               g198(.a(new_n293), .b(new_n288), .c(new_n291), .out0(new_n294));
  nor002aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[30] ));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  norb02aa1n02x5               g201(.a(new_n289), .b(new_n288), .out0(new_n297));
  aoai13aa1n02x7               g202(.a(new_n297), .b(new_n272), .c(new_n174), .d(new_n262), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n299));
  aoi012aa1n03x5               g204(.a(new_n296), .b(new_n298), .c(new_n299), .o1(new_n300));
  aobi12aa1n06x5               g205(.a(new_n297), .b(new_n263), .c(new_n265), .out0(new_n301));
  nano22aa1n03x7               g206(.a(new_n301), .b(new_n296), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[31] ));
  xnrb03aa1n02x5               g208(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g209(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aob012aa1n02x5               g212(.a(new_n122), .b(new_n121), .c(new_n123), .out0(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oabi12aa1n02x5               g214(.a(new_n108), .b(new_n308), .c(new_n101), .out0(new_n310));
  xorc02aa1n02x5               g215(.a(\a[6] ), .b(\b[5] ), .out0(new_n311));
  aoi122aa1n02x5               g216(.a(new_n101), .b(new_n107), .c(new_n106), .d(new_n308), .e(new_n311), .o1(new_n312));
  norb02aa1n02x5               g217(.a(new_n310), .b(new_n312), .out0(\s[7] ));
  xobna2aa1n03x5               g218(.a(new_n103), .b(new_n310), .c(new_n106), .out0(\s[8] ));
  xnrc02aa1n02x5               g219(.a(new_n130), .b(new_n126), .out0(\s[9] ));
endmodule


