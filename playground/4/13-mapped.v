// Benchmark "adder" written by ABC on Wed Jul 17 14:03:33 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n320,
    new_n321, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor002aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[8] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[4] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[3] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  nor042aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand02aa1n04x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aoi012aa1n06x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  aboi22aa1n03x5               g014(.a(\b[3] ), .b(new_n100), .c(new_n101), .d(new_n102), .out0(new_n110));
  oai012aa1n04x7               g015(.a(new_n110), .b(new_n109), .c(new_n105), .o1(new_n111));
  oaib12aa1n06x5               g016(.a(new_n111), .b(new_n100), .c(\b[3] ), .out0(new_n112));
  nor002aa1n03x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand02aa1d04x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nor002aa1d32x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand42aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  tech160nm_fixnrc02aa1n02p5x5 g022(.a(\b[5] ), .b(\a[6] ), .out0(new_n118));
  xorc02aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  nona22aa1n09x5               g024(.a(new_n119), .b(new_n117), .c(new_n118), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  oai022aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n121), .o1(new_n123));
  inv000aa1d42x5               g028(.a(new_n115), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  oabi12aa1n06x5               g030(.a(new_n125), .b(new_n117), .c(new_n123), .out0(new_n126));
  oabi12aa1n06x5               g031(.a(new_n126), .b(new_n120), .c(new_n112), .out0(new_n127));
  oaib12aa1n06x5               g032(.a(new_n127), .b(new_n99), .c(\a[9] ), .out0(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n98), .out0(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g035(.a(\b[9] ), .o1(new_n131));
  tech160nm_fixorc02aa1n05x5   g036(.a(\a[10] ), .b(\b[9] ), .out0(new_n132));
  oai112aa1n03x5               g037(.a(new_n128), .b(new_n132), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  nor002aa1d32x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  oai112aa1n03x5               g041(.a(new_n133), .b(new_n136), .c(new_n131), .d(new_n97), .o1(new_n137));
  oaoi13aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n97), .d(new_n131), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  nor022aa1n16x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n09x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  nona22aa1n02x4               g047(.a(new_n137), .b(new_n142), .c(new_n134), .out0(new_n143));
  inv040aa1n03x5               g048(.a(new_n134), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n142), .o1(new_n145));
  tech160nm_fiaoi012aa1n02p5x5 g050(.a(new_n145), .b(new_n137), .c(new_n144), .o1(new_n146));
  norb02aa1n03x4               g051(.a(new_n143), .b(new_n146), .out0(\s[12] ));
  xnrc02aa1n12x5               g052(.a(\b[12] ), .b(\a[13] ), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nona23aa1n09x5               g054(.a(new_n141), .b(new_n135), .c(new_n134), .d(new_n140), .out0(new_n150));
  oaoi03aa1n12x5               g055(.a(new_n97), .b(new_n131), .c(new_n98), .o1(new_n151));
  tech160nm_fioaoi03aa1n02p5x5 g056(.a(\a[12] ), .b(\b[11] ), .c(new_n144), .o1(new_n152));
  oabi12aa1n18x5               g057(.a(new_n152), .b(new_n150), .c(new_n151), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  and002aa1n02x5               g059(.a(\b[3] ), .b(\a[4] ), .o(new_n155));
  oaoi13aa1n09x5               g060(.a(new_n155), .b(new_n110), .c(new_n109), .d(new_n105), .o1(new_n156));
  norb03aa1n03x5               g061(.a(new_n119), .b(new_n117), .c(new_n118), .out0(new_n157));
  nano23aa1n06x5               g062(.a(new_n134), .b(new_n140), .c(new_n141), .d(new_n135), .out0(new_n158));
  xorc02aa1n12x5               g063(.a(\a[9] ), .b(\b[8] ), .out0(new_n159));
  nand23aa1n06x5               g064(.a(new_n158), .b(new_n132), .c(new_n159), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n03x5               g066(.a(new_n161), .b(new_n126), .c(new_n156), .d(new_n157), .o1(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n149), .b(new_n162), .c(new_n154), .out0(\s[13] ));
  orn002aa1n03x5               g068(.a(\a[13] ), .b(\b[12] ), .o(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n148), .c(new_n162), .d(new_n154), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .out0(new_n167));
  nor042aa1n06x5               g072(.a(new_n167), .b(new_n148), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  oao003aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n164), .carry(new_n170));
  aoai13aa1n04x5               g075(.a(new_n170), .b(new_n169), .c(new_n162), .d(new_n154), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1d28x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nor042aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand42aa1n08x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoi112aa1n02x5               g082(.a(new_n177), .b(new_n173), .c(new_n171), .d(new_n174), .o1(new_n178));
  aoai13aa1n04x5               g083(.a(new_n177), .b(new_n173), .c(new_n171), .d(new_n174), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  nano23aa1n09x5               g085(.a(new_n173), .b(new_n175), .c(new_n176), .d(new_n174), .out0(new_n181));
  nona22aa1n09x5               g086(.a(new_n181), .b(new_n167), .c(new_n148), .out0(new_n182));
  nor042aa1n06x5               g087(.a(new_n182), .b(new_n160), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n126), .c(new_n156), .d(new_n157), .o1(new_n184));
  aoi012aa1n02x5               g089(.a(new_n175), .b(new_n173), .c(new_n176), .o1(new_n185));
  oaib12aa1n03x5               g090(.a(new_n185), .b(new_n170), .c(new_n181), .out0(new_n186));
  aoi013aa1n06x4               g091(.a(new_n186), .b(new_n153), .c(new_n168), .d(new_n181), .o1(new_n187));
  nand02aa1d06x5               g092(.a(new_n184), .b(new_n187), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g094(.a(\a[18] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[16] ), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(new_n191), .b(new_n192), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  xroi22aa1d04x5               g099(.a(new_n191), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n195));
  nand02aa1d04x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nona22aa1n02x4               g101(.a(new_n196), .b(\b[16] ), .c(\a[17] ), .out0(new_n197));
  oaib12aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n190), .out0(new_n198));
  nor042aa1n03x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand42aa1n03x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n198), .c(new_n188), .d(new_n195), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n198), .c(new_n188), .d(new_n195), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n03x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n04x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona22aa1n02x5               g113(.a(new_n202), .b(new_n208), .c(new_n199), .out0(new_n209));
  orn002aa1n02x5               g114(.a(\a[19] ), .b(\b[18] ), .o(new_n210));
  aobi12aa1n06x5               g115(.a(new_n208), .b(new_n202), .c(new_n210), .out0(new_n211));
  norb02aa1n03x4               g116(.a(new_n209), .b(new_n211), .out0(\s[20] ));
  nano23aa1n03x5               g117(.a(new_n199), .b(new_n206), .c(new_n207), .d(new_n200), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n195), .b(new_n213), .o1(new_n214));
  norp02aa1n02x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  aoi013aa1n06x5               g120(.a(new_n215), .b(new_n196), .c(new_n191), .d(new_n192), .o1(new_n216));
  nona23aa1n09x5               g121(.a(new_n207), .b(new_n200), .c(new_n199), .d(new_n206), .out0(new_n217));
  aoi012aa1n06x5               g122(.a(new_n206), .b(new_n199), .c(new_n207), .o1(new_n218));
  oai012aa1d24x5               g123(.a(new_n218), .b(new_n217), .c(new_n216), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n04x5               g125(.a(new_n220), .b(new_n214), .c(new_n184), .d(new_n187), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n02x7               g132(.a(new_n227), .b(new_n226), .out0(\s[22] ));
  inv000aa1d42x5               g133(.a(\a[21] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\a[22] ), .o1(new_n230));
  xroi22aa1d06x4               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(\b[21] ), .o1(new_n232));
  oaoi03aa1n09x5               g137(.a(new_n230), .b(new_n232), .c(new_n223), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n219), .c(new_n231), .o1(new_n235));
  nanp03aa1n02x5               g140(.a(new_n231), .b(new_n195), .c(new_n213), .o1(new_n236));
  aoai13aa1n04x5               g141(.a(new_n235), .b(new_n236), .c(new_n184), .d(new_n187), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n243));
  norb02aa1n02x7               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  inv000aa1d42x5               g149(.a(\a[23] ), .o1(new_n245));
  inv040aa1d32x5               g150(.a(\a[24] ), .o1(new_n246));
  xroi22aa1d06x4               g151(.a(new_n245), .b(\b[22] ), .c(new_n246), .d(\b[23] ), .out0(new_n247));
  nano22aa1n02x4               g152(.a(new_n214), .b(new_n231), .c(new_n247), .out0(new_n248));
  inv020aa1n02x5               g153(.a(new_n218), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n231), .b(new_n249), .c(new_n213), .d(new_n198), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n247), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n252));
  aoib12aa1n02x5               g157(.a(new_n252), .b(new_n246), .c(\b[23] ), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n251), .c(new_n250), .d(new_n233), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n188), .d(new_n248), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(new_n255), .b(new_n254), .c(new_n188), .d(new_n248), .o1(new_n257));
  norb02aa1n03x4               g162(.a(new_n256), .b(new_n257), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  nona22aa1n02x5               g165(.a(new_n256), .b(new_n260), .c(new_n259), .out0(new_n261));
  inv000aa1n02x5               g166(.a(new_n259), .o1(new_n262));
  aobi12aa1n06x5               g167(.a(new_n260), .b(new_n256), .c(new_n262), .out0(new_n263));
  norb02aa1n03x4               g168(.a(new_n261), .b(new_n263), .out0(\s[26] ));
  oabi12aa1n02x5               g169(.a(new_n186), .b(new_n154), .c(new_n182), .out0(new_n265));
  inv000aa1d42x5               g170(.a(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(\a[26] ), .o1(new_n267));
  xroi22aa1d06x4               g172(.a(new_n266), .b(\b[24] ), .c(new_n267), .d(\b[25] ), .out0(new_n268));
  nano32aa1n03x7               g173(.a(new_n214), .b(new_n268), .c(new_n231), .d(new_n247), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n265), .c(new_n127), .d(new_n183), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n271));
  aobi12aa1n09x5               g176(.a(new_n271), .b(new_n254), .c(new_n268), .out0(new_n272));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n274), .b(new_n273), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n272), .c(new_n270), .out0(\s[27] ));
  inv000aa1n06x5               g181(.a(new_n273), .o1(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  aobi12aa1n06x5               g183(.a(new_n269), .b(new_n184), .c(new_n187), .out0(new_n279));
  aoai13aa1n04x5               g184(.a(new_n247), .b(new_n234), .c(new_n219), .d(new_n231), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n268), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n271), .b(new_n281), .c(new_n280), .d(new_n253), .o1(new_n282));
  oaih12aa1n02x5               g187(.a(new_n274), .b(new_n282), .c(new_n279), .o1(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n278), .b(new_n283), .c(new_n277), .o1(new_n284));
  aobi12aa1n02x7               g189(.a(new_n274), .b(new_n272), .c(new_n270), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n277), .c(new_n278), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(\s[28] ));
  nano22aa1n02x4               g192(.a(new_n278), .b(new_n277), .c(new_n274), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n282), .c(new_n279), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n288), .b(new_n272), .c(new_n270), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n275), .b(new_n291), .c(new_n278), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n297), .b(new_n282), .c(new_n279), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n297), .b(new_n272), .c(new_n270), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb03aa1n02x5               g209(.a(new_n288), .b(new_n300), .c(new_n291), .out0(new_n305));
  aobi12aa1n02x7               g210(.a(new_n305), .b(new_n272), .c(new_n270), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n306), .b(new_n307), .c(new_n308), .out0(new_n309));
  oaih12aa1n02x5               g214(.a(new_n305), .b(new_n282), .c(new_n279), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n308), .b(new_n310), .c(new_n307), .o1(new_n311));
  norp02aa1n03x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnbna2aa1n03x5               g217(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n156), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g221(.a(\a[5] ), .b(\b[4] ), .c(new_n112), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g223(.a(new_n115), .b(new_n116), .out0(new_n319));
  inv000aa1d42x5               g224(.a(new_n319), .o1(new_n320));
  oai112aa1n02x5               g225(.a(new_n121), .b(new_n320), .c(new_n317), .d(new_n118), .o1(new_n321));
  oaoi13aa1n02x5               g226(.a(new_n320), .b(new_n121), .c(new_n317), .d(new_n118), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n321), .b(new_n322), .out0(\s[7] ));
  nanb02aa1n02x5               g228(.a(new_n113), .b(new_n114), .out0(new_n324));
  xobna2aa1n03x5               g229(.a(new_n324), .b(new_n321), .c(new_n124), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


