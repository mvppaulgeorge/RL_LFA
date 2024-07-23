// Benchmark "adder" written by ABC on Thu Jul 18 03:02:44 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n340, new_n341, new_n343, new_n344, new_n347,
    new_n348, new_n349, new_n350, new_n352, new_n353, new_n354, new_n356,
    new_n357, new_n359, new_n360;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  nand42aa1n08x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor042aa1n04x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nor002aa1n04x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  oai012aa1n04x7               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norb02aa1n06x4               g007(.a(new_n99), .b(new_n101), .out0(new_n103));
  and002aa1n12x5               g008(.a(\b[0] ), .b(\a[1] ), .o(new_n104));
  oaoi03aa1n12x5               g009(.a(\a[2] ), .b(\b[1] ), .c(new_n104), .o1(new_n105));
  nand42aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n06x5               g011(.a(new_n106), .b(new_n100), .out0(new_n107));
  nanp03aa1d12x5               g012(.a(new_n105), .b(new_n103), .c(new_n107), .o1(new_n108));
  inv040aa1d32x5               g013(.a(\a[7] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[6] ), .o1(new_n110));
  oai022aa1n02x5               g015(.a(new_n109), .b(new_n110), .c(\b[7] ), .d(\a[8] ), .o1(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  oai012aa1n02x5               g017(.a(new_n112), .b(\b[6] ), .c(\a[7] ), .o1(new_n113));
  xorc02aa1n06x5               g018(.a(\a[6] ), .b(\b[5] ), .out0(new_n114));
  xorc02aa1n12x5               g019(.a(\a[5] ), .b(\b[4] ), .out0(new_n115));
  nona23aa1d18x5               g020(.a(new_n114), .b(new_n115), .c(new_n113), .d(new_n111), .out0(new_n116));
  nor042aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  and002aa1n03x5               g022(.a(\b[7] ), .b(\a[8] ), .o(new_n118));
  aoi112aa1n03x5               g023(.a(new_n118), .b(new_n117), .c(new_n109), .d(new_n110), .o1(new_n119));
  oai022aa1n03x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aoi022aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n121));
  nand42aa1n03x5               g026(.a(new_n110), .b(new_n109), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  aoi013aa1n06x4               g028(.a(new_n123), .b(new_n119), .c(new_n120), .d(new_n121), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n116), .c(new_n108), .d(new_n102), .o1(new_n125));
  xnrc02aa1n12x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(new_n125), .b(new_n127), .o1(new_n128));
  tech160nm_fixorc02aa1n03p5x5 g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  oao003aa1n02x5               g035(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .carry(new_n131));
  norb02aa1n02x5               g036(.a(new_n129), .b(new_n126), .out0(new_n132));
  nand22aa1n03x5               g037(.a(new_n125), .b(new_n132), .o1(new_n133));
  nor002aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n03x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n133), .c(new_n131), .out0(\s[11] ));
  aob012aa1n03x5               g042(.a(new_n136), .b(new_n133), .c(new_n131), .out0(new_n138));
  nor042aa1n03x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  aoib12aa1n02x5               g046(.a(new_n134), .b(new_n140), .c(new_n139), .out0(new_n142));
  oaih12aa1n02x5               g047(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .o1(new_n143));
  aoi022aa1n03x5               g048(.a(new_n143), .b(new_n141), .c(new_n138), .d(new_n142), .o1(\s[12] ));
  nano32aa1n06x5               g049(.a(new_n126), .b(new_n141), .c(new_n129), .d(new_n136), .out0(new_n145));
  oai022aa1n02x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  nano22aa1n03x5               g051(.a(new_n139), .b(new_n135), .c(new_n140), .out0(new_n147));
  aoi012aa1n02x5               g052(.a(new_n134), .b(\a[10] ), .c(\b[9] ), .o1(new_n148));
  nanp03aa1n03x5               g053(.a(new_n147), .b(new_n146), .c(new_n148), .o1(new_n149));
  tech160nm_fiaoi012aa1n03p5x5 g054(.a(new_n139), .b(new_n134), .c(new_n140), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n149), .b(new_n150), .o1(new_n151));
  nor042aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n151), .c(new_n125), .d(new_n145), .o1(new_n156));
  nanp03aa1n02x5               g061(.a(new_n149), .b(new_n150), .c(new_n154), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n157), .b(new_n125), .c(new_n145), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n156), .b(new_n158), .out0(\s[13] ));
  orn002aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .o(new_n160));
  nor042aa1n04x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n09x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n156), .c(new_n160), .out0(\s[14] ));
  nano23aa1n06x5               g069(.a(new_n152), .b(new_n161), .c(new_n162), .d(new_n153), .out0(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n151), .c(new_n125), .d(new_n145), .o1(new_n166));
  oai012aa1n02x5               g071(.a(new_n162), .b(new_n161), .c(new_n152), .o1(new_n167));
  nor042aa1d18x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1d21x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n166), .c(new_n167), .out0(\s[15] ));
  aob012aa1n03x5               g076(.a(new_n170), .b(new_n166), .c(new_n167), .out0(new_n172));
  tech160nm_fixorc02aa1n04x5   g077(.a(\a[16] ), .b(\b[15] ), .out0(new_n173));
  inv000aa1d42x5               g078(.a(\a[16] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[15] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  aoi012aa1n02x5               g082(.a(new_n168), .b(new_n176), .c(new_n177), .o1(new_n178));
  inv040aa1n03x5               g083(.a(new_n168), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n170), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n166), .d(new_n167), .o1(new_n181));
  aoi022aa1n03x5               g086(.a(new_n181), .b(new_n173), .c(new_n172), .d(new_n178), .o1(\s[16] ));
  nano23aa1n02x4               g087(.a(new_n134), .b(new_n139), .c(new_n140), .d(new_n135), .out0(new_n183));
  nand23aa1n03x5               g088(.a(new_n165), .b(new_n170), .c(new_n173), .o1(new_n184));
  nano32aa1n03x7               g089(.a(new_n184), .b(new_n183), .c(new_n127), .d(new_n129), .out0(new_n185));
  nanp02aa1n06x5               g090(.a(new_n125), .b(new_n185), .o1(new_n186));
  nanp03aa1n02x5               g091(.a(new_n176), .b(new_n169), .c(new_n177), .o1(new_n187));
  oai112aa1n03x5               g092(.a(new_n179), .b(new_n162), .c(new_n161), .d(new_n152), .o1(new_n188));
  nor022aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  tech160nm_fioaoi03aa1n02p5x5 g094(.a(new_n174), .b(new_n175), .c(new_n168), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n09x5               g096(.a(new_n191), .b(new_n184), .c(new_n149), .d(new_n150), .o1(new_n192));
  nanb02aa1n06x5               g097(.a(new_n192), .b(new_n186), .out0(new_n193));
  tech160nm_fixorc02aa1n05x5   g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  nona22aa1n02x4               g099(.a(new_n190), .b(new_n189), .c(new_n194), .out0(new_n195));
  aoib12aa1n02x5               g100(.a(new_n195), .b(new_n151), .c(new_n184), .out0(new_n196));
  aoi022aa1n02x5               g101(.a(new_n193), .b(new_n194), .c(new_n186), .d(new_n196), .o1(\s[17] ));
  nor002aa1d32x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  aoai13aa1n02x5               g104(.a(new_n194), .b(new_n192), .c(new_n125), .d(new_n185), .o1(new_n200));
  nor042aa1n06x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand02aa1d06x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  norb02aa1n06x4               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n200), .c(new_n199), .out0(\s[18] ));
  and002aa1n02x5               g109(.a(new_n194), .b(new_n203), .o(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n192), .c(new_n125), .d(new_n185), .o1(new_n206));
  oaoi03aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  nor002aa1d32x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand42aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  norb02aa1n12x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n206), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n03x5               g118(.a(new_n211), .b(new_n206), .c(new_n208), .out0(new_n214));
  nor002aa1n20x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand02aa1d12x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n02x7               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  inv000aa1d42x5               g122(.a(\a[19] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\b[18] ), .o1(new_n219));
  aboi22aa1n03x5               g124(.a(new_n215), .b(new_n216), .c(new_n218), .d(new_n219), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n209), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n211), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n221), .b(new_n222), .c(new_n206), .d(new_n208), .o1(new_n223));
  aoi022aa1n02x5               g128(.a(new_n223), .b(new_n217), .c(new_n214), .d(new_n220), .o1(\s[20] ));
  nano32aa1n03x7               g129(.a(new_n222), .b(new_n194), .c(new_n217), .d(new_n203), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n192), .c(new_n125), .d(new_n185), .o1(new_n226));
  nanb03aa1n06x5               g131(.a(new_n215), .b(new_n216), .c(new_n210), .out0(new_n227));
  oai112aa1n06x5               g132(.a(new_n221), .b(new_n202), .c(new_n201), .d(new_n198), .o1(new_n228));
  aoi012aa1d24x5               g133(.a(new_n215), .b(new_n209), .c(new_n216), .o1(new_n229));
  oai012aa1n18x5               g134(.a(new_n229), .b(new_n228), .c(new_n227), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nor002aa1d32x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nand02aa1n04x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1d27x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  aob012aa1n03x5               g139(.a(new_n234), .b(new_n226), .c(new_n231), .out0(new_n235));
  nano22aa1n03x5               g140(.a(new_n215), .b(new_n210), .c(new_n216), .out0(new_n236));
  oai012aa1n02x7               g141(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .o1(new_n237));
  oab012aa1n04x5               g142(.a(new_n237), .b(new_n198), .c(new_n201), .out0(new_n238));
  inv020aa1n03x5               g143(.a(new_n229), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(new_n239), .b(new_n234), .c(new_n238), .d(new_n236), .o1(new_n240));
  aobi12aa1n02x7               g145(.a(new_n235), .b(new_n240), .c(new_n226), .out0(\s[21] ));
  nor042aa1n04x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nand02aa1n06x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  aoib12aa1n02x5               g149(.a(new_n232), .b(new_n243), .c(new_n242), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n232), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n234), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n246), .b(new_n247), .c(new_n226), .d(new_n231), .o1(new_n248));
  aoi022aa1n03x5               g153(.a(new_n248), .b(new_n244), .c(new_n235), .d(new_n245), .o1(\s[22] ));
  inv000aa1n02x5               g154(.a(new_n225), .o1(new_n250));
  nano22aa1n02x4               g155(.a(new_n250), .b(new_n234), .c(new_n244), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n192), .c(new_n125), .d(new_n185), .o1(new_n252));
  nano23aa1d15x5               g157(.a(new_n232), .b(new_n242), .c(new_n243), .d(new_n233), .out0(new_n253));
  aoi012aa1d18x5               g158(.a(new_n242), .b(new_n232), .c(new_n243), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoi012aa1n02x5               g160(.a(new_n255), .b(new_n230), .c(new_n253), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[23] ), .b(\b[22] ), .out0(new_n257));
  aob012aa1n03x5               g162(.a(new_n257), .b(new_n252), .c(new_n256), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n257), .b(new_n255), .c(new_n230), .d(new_n253), .o1(new_n259));
  aobi12aa1n02x7               g164(.a(new_n258), .b(new_n259), .c(new_n252), .out0(\s[23] ));
  xorc02aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .out0(new_n261));
  nor042aa1n06x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  norp02aa1n02x5               g167(.a(new_n261), .b(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n262), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n257), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n264), .b(new_n265), .c(new_n252), .d(new_n256), .o1(new_n266));
  aoi022aa1n03x5               g171(.a(new_n266), .b(new_n261), .c(new_n258), .d(new_n263), .o1(\s[24] ));
  and002aa1n06x5               g172(.a(new_n261), .b(new_n257), .o(new_n268));
  nano22aa1n02x4               g173(.a(new_n250), .b(new_n268), .c(new_n253), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n192), .c(new_n125), .d(new_n185), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n253), .b(new_n239), .c(new_n238), .d(new_n236), .o1(new_n271));
  inv000aa1n06x5               g176(.a(new_n268), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .carry(new_n273));
  aoai13aa1n12x5               g178(.a(new_n273), .b(new_n272), .c(new_n271), .d(new_n254), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[25] ), .b(\b[24] ), .out0(new_n276));
  aob012aa1n03x5               g181(.a(new_n276), .b(new_n270), .c(new_n275), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n268), .b(new_n255), .c(new_n230), .d(new_n253), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n276), .o1(new_n279));
  and003aa1n02x5               g184(.a(new_n278), .b(new_n279), .c(new_n273), .o(new_n280));
  aobi12aa1n02x7               g185(.a(new_n277), .b(new_n280), .c(new_n270), .out0(\s[25] ));
  tech160nm_fixorc02aa1n04x5   g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  nor042aa1n03x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n282), .b(new_n283), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n283), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n279), .c(new_n270), .d(new_n275), .o1(new_n286));
  aoi022aa1n03x5               g191(.a(new_n286), .b(new_n282), .c(new_n277), .d(new_n284), .o1(\s[26] ));
  and002aa1n12x5               g192(.a(new_n282), .b(new_n276), .o(new_n288));
  nano32aa1n03x7               g193(.a(new_n250), .b(new_n288), .c(new_n253), .d(new_n268), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n192), .c(new_n125), .d(new_n185), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n288), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[26] ), .b(\b[25] ), .c(new_n285), .carry(new_n292));
  aoai13aa1n04x5               g197(.a(new_n292), .b(new_n291), .c(new_n278), .d(new_n273), .o1(new_n293));
  xorc02aa1n12x5               g198(.a(\a[27] ), .b(\b[26] ), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n293), .c(new_n193), .d(new_n289), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n292), .o1(new_n296));
  aoi112aa1n02x5               g201(.a(new_n294), .b(new_n296), .c(new_n274), .d(new_n288), .o1(new_n297));
  aobi12aa1n02x7               g202(.a(new_n295), .b(new_n297), .c(new_n290), .out0(\s[27] ));
  tech160nm_fixorc02aa1n02p5x5 g203(.a(\a[28] ), .b(\b[27] ), .out0(new_n299));
  norp02aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  norp02aa1n02x5               g205(.a(new_n299), .b(new_n300), .o1(new_n301));
  tech160nm_fiaoi012aa1n05x5   g206(.a(new_n296), .b(new_n274), .c(new_n288), .o1(new_n302));
  inv000aa1n03x5               g207(.a(new_n300), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n294), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n303), .b(new_n304), .c(new_n302), .d(new_n290), .o1(new_n305));
  aoi022aa1n03x5               g210(.a(new_n305), .b(new_n299), .c(new_n295), .d(new_n301), .o1(\s[28] ));
  and002aa1n02x5               g211(.a(new_n299), .b(new_n294), .o(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n293), .c(new_n193), .d(new_n289), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .out0(new_n309));
  oao003aa1n02x5               g214(.a(\a[28] ), .b(\b[27] ), .c(new_n303), .carry(new_n310));
  norb02aa1n02x5               g215(.a(new_n310), .b(new_n309), .out0(new_n311));
  inv000aa1d42x5               g216(.a(new_n307), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n310), .b(new_n312), .c(new_n302), .d(new_n290), .o1(new_n313));
  aoi022aa1n03x5               g218(.a(new_n313), .b(new_n309), .c(new_n308), .d(new_n311), .o1(\s[29] ));
  xnrb03aa1n02x5               g219(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d24x5               g220(.a(new_n304), .b(new_n299), .c(new_n309), .out0(new_n316));
  aoai13aa1n02x7               g221(.a(new_n316), .b(new_n293), .c(new_n193), .d(new_n289), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  inv000aa1d42x5               g223(.a(\b[28] ), .o1(new_n319));
  inv000aa1d42x5               g224(.a(\a[29] ), .o1(new_n320));
  oaib12aa1n02x5               g225(.a(new_n310), .b(\b[28] ), .c(new_n320), .out0(new_n321));
  oaib12aa1n02x5               g226(.a(new_n321), .b(new_n319), .c(\a[29] ), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n318), .c(new_n302), .d(new_n290), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .out0(new_n324));
  oaoi13aa1n02x5               g229(.a(new_n324), .b(new_n321), .c(new_n320), .d(new_n319), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n323), .b(new_n324), .c(new_n317), .d(new_n325), .o1(\s[30] ));
  nano32aa1d15x5               g231(.a(new_n304), .b(new_n324), .c(new_n299), .d(new_n309), .out0(new_n327));
  aoai13aa1n02x7               g232(.a(new_n327), .b(new_n293), .c(new_n193), .d(new_n289), .o1(new_n328));
  aoi022aa1n02x5               g233(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n329));
  norb02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  obai22aa1n02x7               g235(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n331));
  aoi112aa1n02x5               g236(.a(new_n331), .b(new_n330), .c(new_n321), .d(new_n329), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  inv000aa1d42x5               g238(.a(new_n327), .o1(new_n334));
  norp02aa1n02x5               g239(.a(\b[29] ), .b(\a[30] ), .o1(new_n335));
  aoi012aa1n02x5               g240(.a(new_n335), .b(new_n321), .c(new_n329), .o1(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n334), .c(new_n302), .d(new_n290), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n333), .c(new_n328), .d(new_n332), .o1(\s[31] ));
  orn002aa1n02x5               g243(.a(\a[2] ), .b(\b[1] ), .o(new_n339));
  nanp02aa1n02x5               g244(.a(\b[1] ), .b(\a[2] ), .o1(new_n340));
  nanb03aa1n02x5               g245(.a(new_n104), .b(new_n339), .c(new_n340), .out0(new_n341));
  xnbna2aa1n03x5               g246(.a(new_n107), .b(new_n341), .c(new_n339), .out0(\s[3] ));
  aoai13aa1n02x5               g247(.a(new_n103), .b(new_n100), .c(new_n105), .d(new_n106), .o1(new_n343));
  aoi112aa1n02x5               g248(.a(new_n100), .b(new_n103), .c(new_n105), .d(new_n106), .o1(new_n344));
  norb02aa1n02x5               g249(.a(new_n343), .b(new_n344), .out0(\s[4] ));
  xnbna2aa1n03x5               g250(.a(new_n115), .b(new_n108), .c(new_n102), .out0(\s[5] ));
  nanp02aa1n02x5               g251(.a(new_n108), .b(new_n102), .o1(new_n347));
  norp02aa1n02x5               g252(.a(\b[4] ), .b(\a[5] ), .o1(new_n348));
  aoai13aa1n02x5               g253(.a(new_n114), .b(new_n348), .c(new_n347), .d(new_n115), .o1(new_n349));
  aoi112aa1n02x5               g254(.a(new_n348), .b(new_n114), .c(new_n347), .d(new_n115), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n349), .b(new_n350), .out0(\s[6] ));
  aob012aa1n02x5               g256(.a(new_n120), .b(\b[5] ), .c(\a[6] ), .out0(new_n352));
  nanp03aa1n02x5               g257(.a(new_n347), .b(new_n114), .c(new_n115), .o1(new_n353));
  xorc02aa1n02x5               g258(.a(\a[7] ), .b(\b[6] ), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n353), .c(new_n352), .out0(\s[7] ));
  norb02aa1n02x5               g260(.a(new_n112), .b(new_n117), .out0(new_n356));
  aob012aa1n02x5               g261(.a(new_n354), .b(new_n353), .c(new_n352), .out0(new_n357));
  xnbna2aa1n03x5               g262(.a(new_n356), .b(new_n357), .c(new_n122), .out0(\s[8] ));
  nanb02aa1n02x5               g263(.a(new_n116), .b(new_n347), .out0(new_n359));
  aoi113aa1n02x5               g264(.a(new_n123), .b(new_n127), .c(new_n119), .d(new_n120), .e(new_n121), .o1(new_n360));
  aoi022aa1n02x5               g265(.a(new_n360), .b(new_n359), .c(new_n125), .d(new_n127), .o1(\s[9] ));
endmodule


