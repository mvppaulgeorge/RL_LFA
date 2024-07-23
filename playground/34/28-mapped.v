// Benchmark "adder" written by ABC on Thu Jul 18 05:37:34 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n320, new_n322, new_n323, new_n325, new_n327,
    new_n328, new_n330, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n09x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nand42aa1d28x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  nanb02aa1n02x5               g004(.a(new_n98), .b(new_n99), .out0(new_n100));
  nor042aa1n06x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  tech160nm_finor002aa1n03p5x5 g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand02aa1d04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  tech160nm_fiaoi012aa1n05x5   g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor002aa1n20x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand22aa1n12x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  aoi012aa1n02x7               g015(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n111));
  oai012aa1n09x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  norp02aa1n04x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1d06x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nor022aa1n16x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nand42aa1n03x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xnrc02aa1n12x5               g022(.a(\b[7] ), .b(\a[8] ), .out0(new_n118));
  nor042aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand42aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nanb02aa1n02x5               g025(.a(new_n119), .b(new_n120), .out0(new_n121));
  nor043aa1n03x5               g026(.a(new_n117), .b(new_n118), .c(new_n121), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n112), .b(new_n122), .o1(new_n123));
  nanb03aa1n03x5               g028(.a(new_n119), .b(new_n120), .c(new_n114), .out0(new_n124));
  nor002aa1n02x5               g029(.a(new_n115), .b(new_n113), .o1(new_n125));
  orn002aa1n02x7               g030(.a(\a[7] ), .b(\b[6] ), .o(new_n126));
  oao003aa1n02x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n126), .carry(new_n127));
  oai013aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n118), .d(new_n125), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n123), .b(new_n128), .c(new_n101), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n100), .b(new_n129), .c(new_n97), .out0(\s[10] ));
  nano22aa1n02x4               g035(.a(new_n98), .b(new_n97), .c(new_n99), .out0(new_n131));
  tech160nm_fiao0012aa1n02p5x5 g036(.a(new_n98), .b(new_n101), .c(new_n99), .o(new_n132));
  nor002aa1n16x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n09x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  aoai13aa1n06x5               g040(.a(new_n135), .b(new_n132), .c(new_n129), .d(new_n131), .o1(new_n136));
  aoi112aa1n02x5               g041(.a(new_n132), .b(new_n135), .c(new_n129), .d(new_n131), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(new_n133), .o1(new_n139));
  nor042aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n136), .c(new_n139), .out0(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n133), .b(new_n140), .c(new_n141), .d(new_n134), .out0(new_n144));
  nano23aa1d12x5               g049(.a(new_n101), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n145));
  nand22aa1n09x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n03x5               g052(.a(new_n147), .b(new_n128), .c(new_n112), .d(new_n122), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n134), .b(new_n98), .c(new_n101), .d(new_n99), .o1(new_n149));
  nona22aa1n02x5               g054(.a(new_n149), .b(new_n140), .c(new_n133), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n141), .o1(new_n151));
  nor022aa1n16x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n151), .out0(\s[13] ));
  nand22aa1n04x5               g060(.a(new_n148), .b(new_n151), .o1(new_n156));
  inv040aa1n02x5               g061(.a(new_n152), .o1(new_n157));
  nor042aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  tech160nm_finand02aa1n03p5x5 g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  oaib12aa1n02x5               g064(.a(new_n157), .b(new_n158), .c(new_n159), .out0(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n156), .c(new_n154), .o1(new_n161));
  nano23aa1n03x7               g066(.a(new_n152), .b(new_n158), .c(new_n159), .d(new_n153), .out0(new_n162));
  nanp02aa1n02x5               g067(.a(new_n156), .b(new_n162), .o1(new_n163));
  oaoi03aa1n12x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n158), .b(new_n163), .c(new_n165), .o1(new_n166));
  norp02aa1n02x5               g071(.a(new_n166), .b(new_n161), .o1(\s[14] ));
  nor022aa1n08x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n163), .c(new_n165), .out0(\s[15] ));
  inv000aa1d42x5               g076(.a(new_n168), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n170), .b(new_n164), .c(new_n156), .d(new_n162), .o1(new_n173));
  nor042aa1n03x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1d28x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n175), .o1(new_n177));
  oai022aa1n02x5               g082(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n178));
  nona22aa1n02x4               g083(.a(new_n173), .b(new_n177), .c(new_n178), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n179), .b(new_n176), .c(new_n173), .d(new_n172), .o1(\s[16] ));
  nano23aa1n02x5               g085(.a(new_n168), .b(new_n174), .c(new_n175), .d(new_n169), .out0(new_n181));
  nano22aa1n03x7               g086(.a(new_n146), .b(new_n162), .c(new_n181), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n128), .c(new_n112), .d(new_n122), .o1(new_n183));
  nano22aa1n03x7               g088(.a(new_n174), .b(new_n169), .c(new_n175), .out0(new_n184));
  oai012aa1n02x5               g089(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .o1(new_n185));
  tech160nm_fioai012aa1n03p5x5 g090(.a(new_n159), .b(\b[14] ), .c(\a[15] ), .o1(new_n186));
  nano23aa1n03x7               g091(.a(new_n186), .b(new_n185), .c(new_n157), .d(new_n141), .out0(new_n187));
  oab012aa1n02x4               g092(.a(new_n186), .b(new_n152), .c(new_n158), .out0(new_n188));
  ao0022aa1n03x5               g093(.a(new_n188), .b(new_n184), .c(new_n178), .d(new_n175), .o(new_n189));
  aoi013aa1n06x4               g094(.a(new_n189), .b(new_n150), .c(new_n184), .d(new_n187), .o1(new_n190));
  nanp02aa1n09x5               g095(.a(new_n183), .b(new_n190), .o1(new_n191));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n181), .b(new_n164), .c(new_n156), .d(new_n162), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n192), .b(new_n175), .c(new_n178), .o1(new_n194));
  aoi022aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n192), .d(new_n191), .o1(\s[17] ));
  norp02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nand02aa1n03x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  obai22aa1n02x7               g102(.a(new_n197), .b(new_n196), .c(\a[17] ), .d(\b[16] ), .out0(new_n198));
  aoi012aa1n02x5               g103(.a(new_n198), .b(new_n191), .c(new_n192), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\a[17] ), .o1(new_n200));
  inv040aa1d32x5               g105(.a(\a[18] ), .o1(new_n201));
  xroi22aa1d06x4               g106(.a(new_n200), .b(\b[16] ), .c(new_n201), .d(\b[17] ), .out0(new_n202));
  oaih22aa1d12x5               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  aoi022aa1n02x5               g108(.a(new_n191), .b(new_n202), .c(new_n197), .d(new_n203), .o1(new_n204));
  oab012aa1n02x4               g109(.a(new_n199), .b(new_n204), .c(new_n196), .out0(\s[18] ));
  and002aa1n02x5               g110(.a(new_n203), .b(new_n197), .o(new_n206));
  xorc02aa1n02x5               g111(.a(\a[19] ), .b(\b[18] ), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n206), .c(new_n191), .d(new_n202), .o1(new_n208));
  aoi112aa1n02x5               g113(.a(new_n207), .b(new_n206), .c(new_n191), .d(new_n202), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g116(.a(\a[19] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\b[18] ), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(new_n213), .b(new_n212), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[20] ), .b(\b[19] ), .out0(new_n215));
  nand02aa1n16x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  oai022aa1n02x5               g122(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n218));
  nona22aa1n03x5               g123(.a(new_n208), .b(new_n217), .c(new_n218), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n215), .c(new_n214), .d(new_n208), .o1(\s[20] ));
  inv000aa1d42x5               g125(.a(\a[20] ), .o1(new_n221));
  xroi22aa1d04x5               g126(.a(new_n212), .b(\b[18] ), .c(new_n221), .d(\b[19] ), .out0(new_n222));
  and002aa1n06x5               g127(.a(new_n222), .b(new_n202), .o(new_n223));
  nanb02aa1n03x5               g128(.a(\b[19] ), .b(new_n221), .out0(new_n224));
  oai112aa1n04x5               g129(.a(new_n224), .b(new_n216), .c(new_n213), .d(new_n212), .o1(new_n225));
  nand03aa1n02x5               g130(.a(new_n203), .b(new_n214), .c(new_n197), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(new_n218), .b(new_n216), .o1(new_n227));
  oai012aa1n06x5               g132(.a(new_n227), .b(new_n226), .c(new_n225), .o1(new_n228));
  xorc02aa1n02x5               g133(.a(\a[21] ), .b(\b[20] ), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n228), .c(new_n191), .d(new_n223), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n228), .c(new_n191), .d(new_n223), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(\s[21] ));
  inv000aa1d42x5               g137(.a(\a[21] ), .o1(new_n233));
  nanb02aa1n02x5               g138(.a(\b[20] ), .b(new_n233), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[22] ), .b(\b[21] ), .out0(new_n235));
  and002aa1n02x5               g140(.a(\b[21] ), .b(\a[22] ), .o(new_n236));
  oai022aa1n02x5               g141(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n237));
  nona22aa1n03x5               g142(.a(new_n230), .b(new_n236), .c(new_n237), .out0(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n235), .c(new_n234), .d(new_n230), .o1(\s[22] ));
  nanp02aa1n02x5               g144(.a(new_n235), .b(new_n229), .o1(new_n240));
  nano22aa1n02x4               g145(.a(new_n240), .b(new_n202), .c(new_n222), .out0(new_n241));
  inv000aa1d42x5               g146(.a(\a[22] ), .o1(new_n242));
  xroi22aa1d04x5               g147(.a(new_n233), .b(\b[20] ), .c(new_n242), .d(\b[21] ), .out0(new_n243));
  oaoi03aa1n02x5               g148(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n244));
  tech160nm_fiao0012aa1n02p5x5 g149(.a(new_n244), .b(new_n228), .c(new_n243), .o(new_n245));
  xorc02aa1n12x5               g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n245), .c(new_n191), .d(new_n241), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n246), .b(new_n245), .c(new_n191), .d(new_n241), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[23] ));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .out0(new_n252));
  and002aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n247), .b(new_n253), .c(new_n254), .out0(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n252), .c(new_n251), .d(new_n247), .o1(\s[24] ));
  nanp02aa1n02x5               g161(.a(new_n252), .b(new_n246), .o1(new_n257));
  nano32aa1n02x4               g162(.a(new_n257), .b(new_n243), .c(new_n222), .d(new_n202), .out0(new_n258));
  inv000aa1n02x5               g163(.a(new_n257), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n244), .c(new_n228), .d(new_n243), .o1(new_n260));
  aob012aa1n02x5               g165(.a(new_n254), .b(\b[23] ), .c(\a[24] ), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  xnrc02aa1n12x5               g167(.a(\b[24] ), .b(\a[25] ), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n262), .c(new_n191), .d(new_n258), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n264), .b(new_n262), .c(new_n191), .d(new_n258), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  and002aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .o(new_n271));
  oai022aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n272));
  nona22aa1n03x5               g177(.a(new_n265), .b(new_n271), .c(new_n272), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n269), .d(new_n265), .o1(\s[26] ));
  nanb02aa1n03x5               g179(.a(new_n263), .b(new_n270), .out0(new_n275));
  nona32aa1n03x5               g180(.a(new_n223), .b(new_n275), .c(new_n257), .d(new_n240), .out0(new_n276));
  aoi012aa1n09x5               g181(.a(new_n276), .b(new_n183), .c(new_n190), .o1(new_n277));
  aob012aa1n02x5               g182(.a(new_n272), .b(\b[25] ), .c(\a[26] ), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n275), .c(new_n260), .d(new_n261), .o1(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  tech160nm_fioai012aa1n05x5   g185(.a(new_n280), .b(new_n279), .c(new_n277), .o1(new_n281));
  nanb02aa1n02x5               g186(.a(new_n280), .b(new_n278), .out0(new_n282));
  aoi113aa1n02x5               g187(.a(new_n277), .b(new_n282), .c(new_n262), .d(new_n264), .e(new_n270), .o1(new_n283));
  norb02aa1n03x4               g188(.a(new_n281), .b(new_n283), .out0(\s[27] ));
  norp02aa1n02x5               g189(.a(\b[26] ), .b(\a[27] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  xorc02aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .out0(new_n287));
  oai022aa1d24x5               g192(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(\a[28] ), .c(\b[27] ), .o1(new_n289));
  tech160nm_finand02aa1n05x5   g194(.a(new_n281), .b(new_n289), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n287), .c(new_n281), .d(new_n286), .o1(\s[28] ));
  xorc02aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .out0(new_n292));
  and002aa1n02x5               g197(.a(new_n287), .b(new_n280), .o(new_n293));
  tech160nm_fioai012aa1n03p5x5 g198(.a(new_n293), .b(new_n279), .c(new_n277), .o1(new_n294));
  inv000aa1d42x5               g199(.a(\b[27] ), .o1(new_n295));
  oaib12aa1n09x5               g200(.a(new_n288), .b(new_n295), .c(\a[28] ), .out0(new_n296));
  nanp03aa1n02x5               g201(.a(new_n294), .b(new_n296), .c(new_n292), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n296), .o1(new_n298));
  oaoi13aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n279), .d(new_n277), .o1(new_n299));
  oai012aa1n03x5               g204(.a(new_n297), .b(new_n299), .c(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g206(.a(new_n280), .b(new_n292), .c(new_n287), .o(new_n302));
  tech160nm_fioaoi03aa1n03p5x5 g207(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .o1(new_n303));
  oaoi13aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n279), .d(new_n277), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .out0(new_n305));
  oai012aa1n02x5               g210(.a(new_n302), .b(new_n279), .c(new_n277), .o1(new_n306));
  norb02aa1n03x5               g211(.a(new_n305), .b(new_n303), .out0(new_n307));
  nanp02aa1n03x5               g212(.a(new_n306), .b(new_n307), .o1(new_n308));
  oai012aa1n03x5               g213(.a(new_n308), .b(new_n304), .c(new_n305), .o1(\s[30] ));
  xorc02aa1n02x5               g214(.a(\a[31] ), .b(\b[30] ), .out0(new_n310));
  and003aa1n02x5               g215(.a(new_n293), .b(new_n305), .c(new_n292), .o(new_n311));
  oai012aa1n02x5               g216(.a(new_n311), .b(new_n279), .c(new_n277), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n307), .b(\a[30] ), .c(\b[29] ), .o1(new_n313));
  norb02aa1n02x5               g218(.a(new_n310), .b(new_n313), .out0(new_n314));
  nand02aa1n02x5               g219(.a(new_n312), .b(new_n314), .o1(new_n315));
  oaoi13aa1n03x5               g220(.a(new_n313), .b(new_n311), .c(new_n279), .d(new_n277), .o1(new_n316));
  oai012aa1n03x5               g221(.a(new_n315), .b(new_n316), .c(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g222(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanb02aa1n02x5               g223(.a(new_n106), .b(new_n107), .out0(new_n319));
  oaib12aa1n02x5               g224(.a(new_n109), .b(new_n108), .c(new_n105), .out0(new_n320));
  aboi22aa1n03x5               g225(.a(new_n106), .b(new_n112), .c(new_n320), .d(new_n319), .out0(\s[4] ));
  orn002aa1n02x5               g226(.a(new_n110), .b(new_n105), .o(new_n322));
  norb02aa1n02x5               g227(.a(new_n116), .b(new_n115), .out0(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n323), .b(new_n322), .c(new_n111), .out0(\s[5] ));
  aoi012aa1n02x5               g229(.a(new_n115), .b(new_n112), .c(new_n116), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g231(.a(new_n117), .b(new_n112), .out0(new_n327));
  oaib12aa1n02x5               g232(.a(new_n327), .b(new_n125), .c(new_n114), .out0(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g234(.a(new_n121), .b(new_n328), .out0(new_n330));
  xobna2aa1n03x5               g235(.a(new_n118), .b(new_n330), .c(new_n126), .out0(\s[8] ));
  aoi012aa1n02x5               g236(.a(new_n128), .b(new_n112), .c(new_n122), .o1(new_n332));
  xnrb03aa1n02x5               g237(.a(new_n332), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


