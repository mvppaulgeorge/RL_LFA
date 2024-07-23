// Benchmark "adder" written by ABC on Thu Jul 18 10:40:46 2024

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
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n306, new_n308, new_n309, new_n310;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n04x5   g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  inv040aa1d30x5               g003(.a(\a[5] ), .o1(new_n99));
  inv040aa1d30x5               g004(.a(\b[4] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[5] ), .b(\a[6] ), .o1(new_n101));
  aoi012aa1n09x5               g006(.a(new_n101), .b(new_n99), .c(new_n100), .o1(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[7] ), .b(\a[8] ), .out0(new_n103));
  inv000aa1d42x5               g008(.a(\a[6] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[5] ), .o1(new_n105));
  orn002aa1n24x5               g010(.a(\a[7] ), .b(\b[6] ), .o(new_n106));
  nand42aa1n06x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  oai112aa1n06x5               g012(.a(new_n106), .b(new_n107), .c(new_n105), .d(new_n104), .o1(new_n108));
  oao003aa1n03x5               g013(.a(\a[8] ), .b(\b[7] ), .c(new_n106), .carry(new_n109));
  oai013aa1n09x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .d(new_n102), .o1(new_n110));
  nor042aa1n03x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand02aa1d04x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nand22aa1n12x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  tech160nm_fiaoi012aa1n04x5   g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  nor002aa1n10x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nand42aa1n02x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  oai012aa1n02x5               g024(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n120));
  oai012aa1n12x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .o1(new_n121));
  nand42aa1n03x5               g026(.a(new_n100), .b(new_n99), .o1(new_n122));
  nand42aa1n03x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oai112aa1n02x5               g028(.a(new_n122), .b(new_n123), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  nor043aa1n06x5               g029(.a(new_n108), .b(new_n124), .c(new_n103), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n110), .c(new_n121), .d(new_n125), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n97), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  oaoi03aa1n06x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n129));
  aoi012aa1d18x5               g034(.a(new_n110), .b(new_n121), .c(new_n125), .o1(new_n130));
  nano22aa1n02x4               g035(.a(new_n130), .b(new_n97), .c(new_n126), .out0(new_n131));
  norp02aa1n02x5               g036(.a(new_n131), .b(new_n129), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv030aa1n02x5               g038(.a(new_n133), .o1(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n132), .b(new_n135), .c(new_n134), .out0(\s[11] ));
  oaoi13aa1n02x5               g041(.a(new_n133), .b(new_n135), .c(new_n131), .d(new_n129), .o1(new_n137));
  xnrb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand42aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nano23aa1n09x5               g045(.a(new_n133), .b(new_n139), .c(new_n140), .d(new_n135), .out0(new_n141));
  nanp03aa1n02x5               g046(.a(new_n141), .b(new_n97), .c(new_n126), .o1(new_n142));
  inv000aa1d42x5               g047(.a(\a[10] ), .o1(new_n143));
  inv000aa1d42x5               g048(.a(\b[9] ), .o1(new_n144));
  norp02aa1n02x5               g049(.a(\b[8] ), .b(\a[9] ), .o1(new_n145));
  oaoi03aa1n02x5               g050(.a(new_n143), .b(new_n144), .c(new_n145), .o1(new_n146));
  nona23aa1n09x5               g051(.a(new_n140), .b(new_n135), .c(new_n133), .d(new_n139), .out0(new_n147));
  oaoi03aa1n06x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n134), .o1(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n147), .c(new_n146), .out0(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n130), .c(new_n142), .out0(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n24x5               g056(.a(\a[13] ), .b(\b[12] ), .o(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aobi12aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .out0(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n02p5x5 g060(.a(\b[13] ), .b(\a[14] ), .out0(new_n156));
  nano22aa1n03x7               g061(.a(new_n156), .b(new_n152), .c(new_n153), .out0(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n148), .c(new_n141), .d(new_n129), .o1(new_n158));
  tech160nm_fioaoi03aa1n04x5   g063(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .o1(new_n159));
  inv000aa1n02x5               g064(.a(new_n159), .o1(new_n160));
  nano22aa1n03x7               g065(.a(new_n147), .b(new_n97), .c(new_n126), .out0(new_n161));
  nano22aa1n02x4               g066(.a(new_n130), .b(new_n161), .c(new_n157), .out0(new_n162));
  nano22aa1n02x4               g067(.a(new_n162), .b(new_n158), .c(new_n160), .out0(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  oaoi03aa1n02x5               g069(.a(\a[15] ), .b(\b[14] ), .c(new_n163), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  tech160nm_fixnrc02aa1n02p5x5 g071(.a(\b[14] ), .b(\a[15] ), .out0(new_n167));
  tech160nm_fixnrc02aa1n02p5x5 g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  nor042aa1n06x5               g073(.a(new_n168), .b(new_n167), .o1(new_n169));
  aoai13aa1n04x5               g074(.a(new_n169), .b(new_n159), .c(new_n149), .d(new_n157), .o1(new_n170));
  oai022aa1n02x5               g075(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n171));
  aob012aa1n02x5               g076(.a(new_n171), .b(\b[15] ), .c(\a[16] ), .out0(new_n172));
  nand03aa1n03x5               g077(.a(new_n161), .b(new_n157), .c(new_n169), .o1(new_n173));
  oai112aa1n06x5               g078(.a(new_n172), .b(new_n170), .c(new_n130), .d(new_n173), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g080(.a(\a[18] ), .o1(new_n176));
  inv040aa1d32x5               g081(.a(\a[17] ), .o1(new_n177));
  inv030aa1d32x5               g082(.a(\b[16] ), .o1(new_n178));
  oaoi03aa1n03x5               g083(.a(new_n177), .b(new_n178), .c(new_n174), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[17] ), .c(new_n176), .out0(\s[18] ));
  inv000aa1d42x5               g085(.a(new_n169), .o1(new_n181));
  aoai13aa1n04x5               g086(.a(new_n172), .b(new_n181), .c(new_n158), .d(new_n160), .o1(new_n182));
  nor042aa1n04x5               g087(.a(new_n130), .b(new_n173), .o1(new_n183));
  xroi22aa1d06x4               g088(.a(new_n177), .b(\b[16] ), .c(new_n176), .d(\b[17] ), .out0(new_n184));
  oai012aa1n02x5               g089(.a(new_n184), .b(new_n183), .c(new_n182), .o1(new_n185));
  oaih22aa1d12x5               g090(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n186));
  oaib12aa1n06x5               g091(.a(new_n186), .b(new_n176), .c(\b[17] ), .out0(new_n187));
  nor042aa1d18x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nand42aa1n08x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n190), .b(new_n185), .c(new_n187), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n02x5               g097(.a(new_n188), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n178), .b(new_n177), .o1(new_n194));
  tech160nm_fioaoi03aa1n03p5x5 g099(.a(\a[18] ), .b(\b[17] ), .c(new_n194), .o1(new_n195));
  aoai13aa1n06x5               g100(.a(new_n190), .b(new_n195), .c(new_n174), .d(new_n184), .o1(new_n196));
  nor022aa1n06x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nand42aa1n06x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  inv020aa1n02x5               g104(.a(new_n199), .o1(new_n200));
  tech160nm_fiaoi012aa1n02p5x5 g105(.a(new_n200), .b(new_n196), .c(new_n193), .o1(new_n201));
  nona22aa1n03x5               g106(.a(new_n196), .b(new_n199), .c(new_n188), .out0(new_n202));
  norb02aa1n03x4               g107(.a(new_n202), .b(new_n201), .out0(\s[20] ));
  nano32aa1n03x7               g108(.a(new_n200), .b(new_n184), .c(new_n193), .d(new_n189), .out0(new_n204));
  oaih12aa1n02x5               g109(.a(new_n204), .b(new_n183), .c(new_n182), .o1(new_n205));
  oaoi03aa1n09x5               g110(.a(\a[20] ), .b(\b[19] ), .c(new_n193), .o1(new_n206));
  inv030aa1n04x5               g111(.a(new_n206), .o1(new_n207));
  nona23aa1n09x5               g112(.a(new_n198), .b(new_n189), .c(new_n188), .d(new_n197), .out0(new_n208));
  oai012aa1d24x5               g113(.a(new_n207), .b(new_n208), .c(new_n187), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  xorc02aa1n02x5               g115(.a(\a[21] ), .b(\b[20] ), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n205), .c(new_n210), .out0(\s[21] ));
  orn002aa1n24x5               g117(.a(\a[21] ), .b(\b[20] ), .o(new_n213));
  aoai13aa1n03x5               g118(.a(new_n211), .b(new_n209), .c(new_n174), .d(new_n204), .o1(new_n214));
  xnrc02aa1n12x5               g119(.a(\b[21] ), .b(\a[22] ), .out0(new_n215));
  aoi012aa1n03x5               g120(.a(new_n215), .b(new_n214), .c(new_n213), .o1(new_n216));
  aobi12aa1n03x5               g121(.a(new_n211), .b(new_n205), .c(new_n210), .out0(new_n217));
  nano22aa1n02x4               g122(.a(new_n217), .b(new_n213), .c(new_n215), .out0(new_n218));
  nor002aa1n02x5               g123(.a(new_n216), .b(new_n218), .o1(\s[22] ));
  nand42aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  nano22aa1n09x5               g125(.a(new_n215), .b(new_n213), .c(new_n220), .out0(new_n221));
  and002aa1n02x5               g126(.a(new_n204), .b(new_n221), .o(new_n222));
  oaih12aa1n02x5               g127(.a(new_n222), .b(new_n183), .c(new_n182), .o1(new_n223));
  tech160nm_fioaoi03aa1n03p5x5 g128(.a(\a[22] ), .b(\b[21] ), .c(new_n213), .o1(new_n224));
  aoi012aa1n02x5               g129(.a(new_n224), .b(new_n209), .c(new_n221), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[22] ), .b(\a[23] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n223), .c(new_n225), .out0(\s[23] ));
  orn002aa1n02x5               g133(.a(\a[23] ), .b(\b[22] ), .o(new_n229));
  inv030aa1n02x5               g134(.a(new_n225), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n227), .b(new_n230), .c(new_n174), .d(new_n222), .o1(new_n231));
  tech160nm_fixnrc02aa1n02p5x5 g136(.a(\b[23] ), .b(\a[24] ), .out0(new_n232));
  aoi012aa1n03x5               g137(.a(new_n232), .b(new_n231), .c(new_n229), .o1(new_n233));
  aoi012aa1n03x5               g138(.a(new_n226), .b(new_n223), .c(new_n225), .o1(new_n234));
  nano22aa1n03x5               g139(.a(new_n234), .b(new_n229), .c(new_n232), .out0(new_n235));
  nor002aa1n02x5               g140(.a(new_n233), .b(new_n235), .o1(\s[24] ));
  nano23aa1n03x7               g141(.a(new_n188), .b(new_n197), .c(new_n198), .d(new_n189), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n221), .b(new_n206), .c(new_n237), .d(new_n195), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n224), .o1(new_n239));
  nor042aa1n03x5               g144(.a(new_n232), .b(new_n226), .o1(new_n240));
  inv000aa1n04x5               g145(.a(new_n240), .o1(new_n241));
  oao003aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .c(new_n229), .carry(new_n242));
  aoai13aa1n12x5               g147(.a(new_n242), .b(new_n241), .c(new_n238), .d(new_n239), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  inv000aa1n02x5               g149(.a(new_n204), .o1(new_n245));
  nano22aa1n02x5               g150(.a(new_n245), .b(new_n221), .c(new_n240), .out0(new_n246));
  oai012aa1n06x5               g151(.a(new_n246), .b(new_n183), .c(new_n182), .o1(new_n247));
  xnrc02aa1n12x5               g152(.a(\b[24] ), .b(\a[25] ), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  xnbna2aa1n03x5               g154(.a(new_n249), .b(new_n247), .c(new_n244), .out0(\s[25] ));
  nor042aa1n03x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n249), .b(new_n243), .c(new_n174), .d(new_n246), .o1(new_n253));
  xnrc02aa1n02x5               g158(.a(\b[25] ), .b(\a[26] ), .out0(new_n254));
  aoi012aa1n03x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .o1(new_n255));
  tech160nm_fiaoi012aa1n05x5   g160(.a(new_n248), .b(new_n247), .c(new_n244), .o1(new_n256));
  nano22aa1n03x5               g161(.a(new_n256), .b(new_n252), .c(new_n254), .out0(new_n257));
  nor002aa1n02x5               g162(.a(new_n255), .b(new_n257), .o1(\s[26] ));
  nor042aa1n04x5               g163(.a(new_n254), .b(new_n248), .o1(new_n259));
  nano32aa1n03x7               g164(.a(new_n245), .b(new_n259), .c(new_n221), .d(new_n240), .out0(new_n260));
  oai012aa1n06x5               g165(.a(new_n260), .b(new_n182), .c(new_n183), .o1(new_n261));
  oao003aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n262));
  aobi12aa1n09x5               g167(.a(new_n262), .b(new_n243), .c(new_n259), .out0(new_n263));
  xorc02aa1n12x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n261), .c(new_n263), .out0(\s[27] ));
  norp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  inv040aa1n03x5               g171(.a(new_n266), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n240), .b(new_n224), .c(new_n209), .d(new_n221), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n259), .o1(new_n269));
  aoai13aa1n04x5               g174(.a(new_n262), .b(new_n269), .c(new_n268), .d(new_n242), .o1(new_n270));
  aoai13aa1n02x7               g175(.a(new_n264), .b(new_n270), .c(new_n174), .d(new_n260), .o1(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aoi012aa1n03x5               g177(.a(new_n272), .b(new_n271), .c(new_n267), .o1(new_n273));
  aobi12aa1n03x5               g178(.a(new_n264), .b(new_n261), .c(new_n263), .out0(new_n274));
  nano22aa1n03x7               g179(.a(new_n274), .b(new_n267), .c(new_n272), .out0(new_n275));
  nor002aa1n02x5               g180(.a(new_n273), .b(new_n275), .o1(\s[28] ));
  xnrc02aa1n02x5               g181(.a(\b[28] ), .b(\a[29] ), .out0(new_n277));
  norb02aa1n02x5               g182(.a(new_n264), .b(new_n272), .out0(new_n278));
  aoai13aa1n02x7               g183(.a(new_n278), .b(new_n270), .c(new_n174), .d(new_n260), .o1(new_n279));
  oao003aa1n02x5               g184(.a(\a[28] ), .b(\b[27] ), .c(new_n267), .carry(new_n280));
  aoi012aa1n03x5               g185(.a(new_n277), .b(new_n279), .c(new_n280), .o1(new_n281));
  aobi12aa1n03x5               g186(.a(new_n278), .b(new_n261), .c(new_n263), .out0(new_n282));
  nano22aa1n03x7               g187(.a(new_n282), .b(new_n277), .c(new_n280), .out0(new_n283));
  nor002aa1n02x5               g188(.a(new_n281), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g190(.a(\b[29] ), .b(\a[30] ), .out0(new_n286));
  norb03aa1n02x5               g191(.a(new_n264), .b(new_n277), .c(new_n272), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n270), .c(new_n174), .d(new_n260), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n280), .carry(new_n289));
  aoi012aa1n03x5               g194(.a(new_n286), .b(new_n288), .c(new_n289), .o1(new_n290));
  aobi12aa1n03x5               g195(.a(new_n287), .b(new_n261), .c(new_n263), .out0(new_n291));
  nano22aa1n03x7               g196(.a(new_n291), .b(new_n286), .c(new_n289), .out0(new_n292));
  nor002aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[30] ));
  xnrc02aa1n02x5               g198(.a(\b[30] ), .b(\a[31] ), .out0(new_n294));
  norb02aa1n02x5               g199(.a(new_n287), .b(new_n286), .out0(new_n295));
  aoai13aa1n02x7               g200(.a(new_n295), .b(new_n270), .c(new_n174), .d(new_n260), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n289), .carry(new_n297));
  aoi012aa1n03x5               g202(.a(new_n294), .b(new_n296), .c(new_n297), .o1(new_n298));
  aobi12aa1n03x5               g203(.a(new_n295), .b(new_n261), .c(new_n263), .out0(new_n299));
  nano22aa1n03x7               g204(.a(new_n299), .b(new_n294), .c(new_n297), .out0(new_n300));
  nor002aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[31] ));
  xnrb03aa1n02x5               g206(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g207(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g209(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aob012aa1n02x5               g210(.a(new_n122), .b(new_n121), .c(new_n123), .out0(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oabi12aa1n02x5               g212(.a(new_n108), .b(new_n306), .c(new_n101), .out0(new_n308));
  xorc02aa1n02x5               g213(.a(\a[6] ), .b(\b[5] ), .out0(new_n309));
  aoi122aa1n02x5               g214(.a(new_n101), .b(new_n107), .c(new_n106), .d(new_n306), .e(new_n309), .o1(new_n310));
  norb02aa1n02x5               g215(.a(new_n308), .b(new_n310), .out0(\s[7] ));
  xobna2aa1n03x5               g216(.a(new_n103), .b(new_n308), .c(new_n106), .out0(\s[8] ));
  xnrc02aa1n02x5               g217(.a(new_n130), .b(new_n126), .out0(\s[9] ));
endmodule


