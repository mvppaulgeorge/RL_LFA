// Benchmark "adder" written by ABC on Thu Jul 18 01:40:26 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n320, new_n323, new_n325, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand02aa1n03x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  oaoi03aa1n06x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  norp02aa1n12x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nanp02aa1n06x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor022aa1n08x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  tech160nm_finand02aa1n03p5x5 g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  inv000aa1d42x5               g015(.a(\a[3] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[2] ), .o1(new_n112));
  aoai13aa1n02x7               g017(.a(new_n107), .b(new_n106), .c(new_n111), .d(new_n112), .o1(new_n113));
  oaih12aa1n06x5               g018(.a(new_n113), .b(new_n110), .c(new_n105), .o1(new_n114));
  norp02aa1n04x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand42aa1n04x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor022aa1n06x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nona23aa1n09x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .out0(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .out0(new_n121));
  nor043aa1n03x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  orn002aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .o(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[6] ), .b(\b[5] ), .c(new_n123), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n125));
  oaib12aa1n06x5               g030(.a(new_n125), .b(new_n119), .c(new_n124), .out0(new_n126));
  nanp02aa1n04x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n100), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n126), .c(new_n114), .d(new_n122), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(new_n100), .b(new_n97), .o1(new_n131));
  aoi022aa1n02x5               g036(.a(new_n129), .b(new_n131), .c(\b[9] ), .d(\a[10] ), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g038(.a(\a[12] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(\a[11] ), .o1(new_n135));
  inv000aa1d42x5               g040(.a(\b[10] ), .o1(new_n136));
  oaoi03aa1n03x5               g041(.a(new_n135), .b(new_n136), .c(new_n132), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(new_n134), .out0(\s[12] ));
  nanp02aa1n02x5               g043(.a(new_n114), .b(new_n122), .o1(new_n139));
  nano23aa1n02x4               g044(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n140));
  aobi12aa1n03x5               g045(.a(new_n125), .b(new_n140), .c(new_n124), .out0(new_n141));
  xorc02aa1n02x5               g046(.a(\a[11] ), .b(\b[10] ), .out0(new_n142));
  xorc02aa1n02x5               g047(.a(\a[12] ), .b(\b[11] ), .out0(new_n143));
  nano23aa1n06x5               g048(.a(new_n97), .b(new_n100), .c(new_n127), .d(new_n98), .out0(new_n144));
  nand03aa1n03x5               g049(.a(new_n144), .b(new_n142), .c(new_n143), .o1(new_n145));
  nanp02aa1n04x5               g050(.a(new_n136), .b(new_n135), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\b[11] ), .o1(new_n147));
  nand42aa1n06x5               g052(.a(new_n147), .b(new_n134), .o1(new_n148));
  oai012aa1d24x5               g053(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n149));
  aoi022aa1n03x5               g054(.a(\b[11] ), .b(\a[12] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n150));
  inv020aa1n03x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n12x5               g056(.a(new_n148), .b(new_n151), .c(new_n149), .d(new_n146), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n145), .c(new_n139), .d(new_n141), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1d18x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  inv000aa1n06x5               g061(.a(new_n156), .o1(new_n157));
  nand02aa1n03x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n03x5               g063(.a(new_n158), .b(new_n156), .out0(new_n159));
  nanp02aa1n02x5               g064(.a(new_n154), .b(new_n159), .o1(new_n160));
  xorc02aa1n12x5               g065(.a(\a[14] ), .b(\b[13] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n157), .out0(\s[14] ));
  and002aa1n02x5               g067(.a(new_n161), .b(new_n159), .o(new_n163));
  oaoi03aa1n09x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n164));
  nor002aa1n16x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand42aa1n16x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n164), .c(new_n154), .d(new_n163), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n168), .b(new_n164), .c(new_n154), .d(new_n163), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  inv000aa1d42x5               g076(.a(new_n165), .o1(new_n172));
  nor002aa1n03x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand42aa1n06x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  nanp03aa1n03x5               g080(.a(new_n169), .b(new_n172), .c(new_n175), .o1(new_n176));
  tech160nm_fiaoi012aa1n04x5   g081(.a(new_n175), .b(new_n169), .c(new_n172), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(\s[16] ));
  nano23aa1n06x5               g083(.a(new_n165), .b(new_n173), .c(new_n174), .d(new_n166), .out0(new_n179));
  nand23aa1n04x5               g084(.a(new_n179), .b(new_n159), .c(new_n161), .o1(new_n180));
  nor042aa1n03x5               g085(.a(new_n180), .b(new_n145), .o1(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n126), .c(new_n114), .d(new_n122), .o1(new_n182));
  aoi022aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n183));
  oai012aa1n02x5               g088(.a(new_n183), .b(new_n164), .c(new_n165), .o1(new_n184));
  tech160nm_fioai012aa1n04x5   g089(.a(new_n184), .b(\b[15] ), .c(\a[16] ), .o1(new_n185));
  aoib12aa1n12x5               g090(.a(new_n185), .b(new_n152), .c(new_n180), .out0(new_n186));
  nanp02aa1n06x5               g091(.a(new_n182), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[18] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d06x4               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  nor042aa1n04x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  aoi112aa1n09x5               g100(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n196));
  nor042aa1n09x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  nor002aa1d32x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand02aa1d16x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n12x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n198), .c(new_n187), .d(new_n194), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n201), .b(new_n198), .c(new_n187), .d(new_n194), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n16x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1d28x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n12x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  nona22aa1n03x5               g113(.a(new_n202), .b(new_n208), .c(new_n199), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n199), .o1(new_n210));
  aobi12aa1n06x5               g115(.a(new_n208), .b(new_n202), .c(new_n210), .out0(new_n211));
  norb02aa1n03x4               g116(.a(new_n209), .b(new_n211), .out0(\s[20] ));
  nona23aa1d18x5               g117(.a(new_n207), .b(new_n200), .c(new_n199), .d(new_n206), .out0(new_n213));
  inv040aa1n08x5               g118(.a(new_n213), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n194), .b(new_n214), .o1(new_n215));
  oaih12aa1n12x5               g120(.a(new_n207), .b(new_n206), .c(new_n199), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n213), .c(new_n197), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n215), .c(new_n182), .d(new_n186), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n12x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n12x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n12x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n03x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  nand02aa1d12x5               g131(.a(new_n223), .b(new_n222), .o1(new_n227));
  nanb03aa1n06x5               g132(.a(new_n227), .b(new_n194), .c(new_n214), .out0(new_n228));
  oai112aa1n06x5               g133(.a(new_n201), .b(new_n208), .c(new_n196), .d(new_n195), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\a[22] ), .o1(new_n230));
  inv040aa1d32x5               g135(.a(\b[21] ), .o1(new_n231));
  oao003aa1n03x5               g136(.a(new_n230), .b(new_n231), .c(new_n221), .carry(new_n232));
  inv040aa1n03x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n12x5               g138(.a(new_n233), .b(new_n227), .c(new_n229), .d(new_n216), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n228), .c(new_n182), .d(new_n186), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  tech160nm_fixorc02aa1n04x5   g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  tech160nm_fixorc02aa1n04x5   g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  aoi112aa1n03x5               g145(.a(new_n238), .b(new_n240), .c(new_n236), .d(new_n239), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n242));
  norb02aa1n02x7               g147(.a(new_n242), .b(new_n241), .out0(\s[24] ));
  and002aa1n02x7               g148(.a(new_n240), .b(new_n239), .o(new_n244));
  inv000aa1d42x5               g149(.a(\a[24] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(\b[23] ), .o1(new_n246));
  oao003aa1n02x5               g151(.a(new_n245), .b(new_n246), .c(new_n238), .carry(new_n247));
  tech160nm_fiaoi012aa1n05x5   g152(.a(new_n247), .b(new_n234), .c(new_n244), .o1(new_n248));
  nona23aa1n03x5               g153(.a(new_n244), .b(new_n194), .c(new_n227), .d(new_n213), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n248), .b(new_n249), .c(new_n182), .d(new_n186), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  tech160nm_fixorc02aa1n05x5   g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  aoi112aa1n03x5               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n256));
  norb02aa1n03x4               g161(.a(new_n256), .b(new_n255), .out0(\s[26] ));
  oao003aa1n02x5               g162(.a(new_n102), .b(new_n103), .c(new_n104), .carry(new_n258));
  nano23aa1n02x4               g163(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n259));
  aobi12aa1n02x5               g164(.a(new_n113), .b(new_n259), .c(new_n258), .out0(new_n260));
  nona22aa1n02x4               g165(.a(new_n140), .b(new_n120), .c(new_n121), .out0(new_n261));
  oaih12aa1n02x5               g166(.a(new_n141), .b(new_n260), .c(new_n261), .o1(new_n262));
  oaoi13aa1n02x5               g167(.a(new_n173), .b(new_n183), .c(new_n164), .d(new_n165), .o1(new_n263));
  oaib12aa1n09x5               g168(.a(new_n263), .b(new_n180), .c(new_n152), .out0(new_n264));
  and002aa1n12x5               g169(.a(new_n254), .b(new_n253), .o(new_n265));
  nano22aa1n03x7               g170(.a(new_n228), .b(new_n244), .c(new_n265), .out0(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n264), .c(new_n262), .d(new_n181), .o1(new_n267));
  aoai13aa1n09x5               g172(.a(new_n265), .b(new_n247), .c(new_n234), .d(new_n244), .o1(new_n268));
  oai022aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n269));
  aob012aa1n02x5               g174(.a(new_n269), .b(\b[25] ), .c(\a[26] ), .out0(new_n270));
  xorc02aa1n12x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  aoi013aa1n03x5               g177(.a(new_n272), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n273));
  inv020aa1n03x5               g178(.a(new_n266), .o1(new_n274));
  aoi012aa1n06x5               g179(.a(new_n274), .b(new_n182), .c(new_n186), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n227), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n244), .b(new_n232), .c(new_n217), .d(new_n276), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n247), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n265), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n270), .b(new_n279), .c(new_n277), .d(new_n278), .o1(new_n280));
  norp03aa1n02x5               g185(.a(new_n280), .b(new_n275), .c(new_n271), .o1(new_n281));
  norp02aa1n02x5               g186(.a(new_n273), .b(new_n281), .o1(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv040aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  xnrc02aa1n12x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n273), .b(new_n284), .c(new_n285), .out0(new_n286));
  tech160nm_fioai012aa1n04x5   g191(.a(new_n271), .b(new_n280), .c(new_n275), .o1(new_n287));
  aoi012aa1n03x5               g192(.a(new_n285), .b(new_n287), .c(new_n284), .o1(new_n288));
  norp02aa1n03x5               g193(.a(new_n288), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  norb02aa1n02x5               g195(.a(new_n271), .b(new_n285), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n291), .b(new_n280), .c(new_n275), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n293));
  aoi012aa1n03x5               g198(.a(new_n290), .b(new_n292), .c(new_n293), .o1(new_n294));
  inv000aa1n02x5               g199(.a(new_n291), .o1(new_n295));
  aoi013aa1n03x5               g200(.a(new_n295), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n296));
  nano22aa1n03x5               g201(.a(new_n296), .b(new_n290), .c(new_n293), .out0(new_n297));
  norp02aa1n03x5               g202(.a(new_n294), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  norb03aa1n12x5               g205(.a(new_n271), .b(new_n290), .c(new_n285), .out0(new_n301));
  oaih12aa1n02x5               g206(.a(new_n301), .b(new_n280), .c(new_n275), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n300), .b(new_n302), .c(new_n303), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n301), .o1(new_n305));
  aoi013aa1n03x5               g210(.a(new_n305), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n300), .c(new_n303), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n304), .b(new_n307), .o1(\s[30] ));
  norb02aa1n02x5               g213(.a(new_n301), .b(new_n300), .out0(new_n309));
  oaih12aa1n02x5               g214(.a(new_n309), .b(new_n280), .c(new_n275), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  aoi012aa1n03x5               g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  inv000aa1n02x5               g218(.a(new_n309), .o1(new_n314));
  aoi013aa1n03x5               g219(.a(new_n314), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n315));
  nano22aa1n03x5               g220(.a(new_n315), .b(new_n311), .c(new_n312), .out0(new_n316));
  norp02aa1n03x5               g221(.a(new_n313), .b(new_n316), .o1(\s[31] ));
  xorb03aa1n02x5               g222(.a(new_n105), .b(\b[2] ), .c(new_n111), .out0(\s[3] ));
  nanb03aa1n02x5               g223(.a(new_n108), .b(new_n258), .c(new_n109), .out0(new_n319));
  aboi22aa1n03x5               g224(.a(new_n106), .b(new_n107), .c(new_n111), .d(new_n112), .out0(new_n320));
  aboi22aa1n03x5               g225(.a(new_n106), .b(new_n114), .c(new_n319), .d(new_n320), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n114), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n260), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g229(.a(\b[5] ), .b(\a[6] ), .o1(new_n325));
  aoib12aa1n02x5               g230(.a(new_n325), .b(new_n323), .c(new_n120), .out0(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g232(.a(\a[7] ), .b(\b[6] ), .c(new_n326), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g234(.a(new_n128), .b(new_n139), .c(new_n141), .out0(\s[9] ));
endmodule


