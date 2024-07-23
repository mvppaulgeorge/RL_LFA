// Benchmark "adder" written by ABC on Wed Jul 17 23:28:50 2024

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
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n340, new_n341, new_n343, new_n344, new_n346,
    new_n347, new_n349, new_n350, new_n351, new_n353, new_n354, new_n355,
    new_n357, new_n358, new_n359, new_n360, new_n362;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixnrc02aa1n04x5   g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  norp02aa1n09x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv040aa1d30x5               g003(.a(\a[2] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aoi012aa1n06x5               g005(.a(new_n100), .b(\a[2] ), .c(\b[1] ), .o1(new_n101));
  oaib12aa1n09x5               g006(.a(new_n101), .b(\b[1] ), .c(new_n99), .out0(new_n102));
  and002aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o(new_n103));
  inv000aa1d42x5               g008(.a(\b[1] ), .o1(new_n104));
  oai022aa1n02x7               g009(.a(new_n99), .b(new_n104), .c(\b[2] ), .d(\a[3] ), .o1(new_n105));
  nona22aa1n09x5               g010(.a(new_n102), .b(new_n105), .c(new_n103), .out0(new_n106));
  oa0022aa1n09x5               g011(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n107));
  nor002aa1d32x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nor042aa1n04x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  aoi112aa1n09x5               g014(.a(new_n109), .b(new_n108), .c(\a[6] ), .d(\b[5] ), .o1(new_n110));
  nand42aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  aob012aa1n12x5               g016(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(new_n112));
  aoi022aa1n06x5               g017(.a(\b[7] ), .b(\a[8] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n113));
  oai022aa1n04x5               g018(.a(\a[6] ), .b(\b[5] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n110), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n115));
  nand42aa1n04x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  oai122aa1n06x5               g021(.a(new_n116), .b(\a[8] ), .c(\b[7] ), .d(\a[7] ), .e(\b[6] ), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  aoi022aa1d24x5               g023(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n119));
  oai012aa1n06x5               g024(.a(new_n119), .b(new_n118), .c(new_n108), .o1(new_n120));
  nor022aa1n04x5               g025(.a(new_n120), .b(new_n117), .o1(new_n121));
  aoi112aa1n03x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  oab012aa1n06x5               g027(.a(new_n122), .b(\a[8] ), .c(\b[7] ), .out0(new_n123));
  norb02aa1n12x5               g028(.a(new_n123), .b(new_n121), .out0(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n115), .c(new_n106), .d(new_n107), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n97), .b(new_n98), .c(new_n125), .d(new_n126), .o1(new_n127));
  norp02aa1n02x5               g032(.a(new_n97), .b(new_n98), .o1(new_n128));
  aob012aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n126), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n127), .b(new_n129), .o1(\s[10] ));
  and002aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o(new_n131));
  nor002aa1n20x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aboi22aa1n03x5               g039(.a(new_n131), .b(new_n129), .c(new_n133), .d(new_n134), .out0(new_n135));
  nona23aa1n06x5               g040(.a(new_n129), .b(new_n134), .c(new_n132), .d(new_n131), .out0(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(\s[11] ));
  nor022aa1n08x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  nor042aa1n02x5               g045(.a(new_n138), .b(new_n132), .o1(new_n141));
  nanp03aa1n02x5               g046(.a(new_n136), .b(new_n139), .c(new_n141), .o1(new_n142));
  aoai13aa1n03x5               g047(.a(new_n142), .b(new_n140), .c(new_n133), .d(new_n136), .o1(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n132), .b(new_n138), .c(new_n139), .d(new_n134), .out0(new_n144));
  nanb03aa1n06x5               g049(.a(new_n97), .b(new_n144), .c(new_n126), .out0(new_n145));
  nanb02aa1n02x5               g050(.a(new_n145), .b(new_n125), .out0(new_n146));
  oab012aa1n02x5               g051(.a(new_n98), .b(\a[10] ), .c(\b[9] ), .out0(new_n147));
  aob012aa1n02x5               g052(.a(new_n134), .b(\b[9] ), .c(\a[10] ), .out0(new_n148));
  tech160nm_fioai012aa1n03p5x5 g053(.a(new_n141), .b(new_n147), .c(new_n148), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n149), .b(new_n139), .o1(new_n150));
  xorc02aa1n02x5               g055(.a(\a[13] ), .b(\b[12] ), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n146), .c(new_n150), .out0(\s[13] ));
  aobi12aa1n02x5               g057(.a(new_n151), .b(new_n146), .c(new_n150), .out0(new_n153));
  inv000aa1d42x5               g058(.a(\a[13] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(\b[12] ), .o1(new_n155));
  nand22aa1n03x5               g060(.a(new_n106), .b(new_n107), .o1(new_n156));
  nanb02aa1n06x5               g061(.a(new_n115), .b(new_n156), .out0(new_n157));
  aoai13aa1n04x5               g062(.a(new_n150), .b(new_n145), .c(new_n157), .d(new_n124), .o1(new_n158));
  oaoi03aa1n02x5               g063(.a(new_n154), .b(new_n155), .c(new_n158), .o1(new_n159));
  nor002aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n12x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  nor042aa1n04x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nona22aa1n02x4               g068(.a(new_n161), .b(new_n160), .c(new_n163), .out0(new_n164));
  oai022aa1n02x5               g069(.a(new_n159), .b(new_n162), .c(new_n164), .d(new_n153), .o1(\s[14] ));
  nand42aa1n03x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nano23aa1n06x5               g071(.a(new_n163), .b(new_n160), .c(new_n161), .d(new_n166), .out0(new_n167));
  tech160nm_fiaoi012aa1n05x5   g072(.a(new_n160), .b(new_n163), .c(new_n161), .o1(new_n168));
  inv000aa1n06x5               g073(.a(new_n168), .o1(new_n169));
  nor002aa1d32x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  tech160nm_finand02aa1n05x5   g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n18x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  inv040aa1d30x5               g077(.a(new_n172), .o1(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n169), .c(new_n158), .d(new_n167), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n173), .b(new_n169), .c(new_n158), .d(new_n167), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n170), .o1(new_n177));
  nor002aa1n12x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1n03x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  norb03aa1n02x5               g085(.a(new_n179), .b(new_n170), .c(new_n178), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n174), .b(new_n181), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n180), .c(new_n177), .d(new_n174), .o1(\s[16] ));
  nano32aa1d12x5               g088(.a(new_n145), .b(new_n180), .c(new_n167), .d(new_n173), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n125), .b(new_n184), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n178), .o1(new_n186));
  aoai13aa1n06x5               g091(.a(new_n179), .b(new_n170), .c(new_n169), .d(new_n171), .o1(new_n187));
  nano22aa1n03x7               g092(.a(new_n178), .b(new_n171), .c(new_n179), .out0(new_n188));
  oai012aa1n02x5               g093(.a(new_n139), .b(\b[12] ), .c(\a[13] ), .o1(new_n189));
  oai012aa1n02x5               g094(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .o1(new_n190));
  oai012aa1n02x5               g095(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .o1(new_n191));
  nor043aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n189), .o1(new_n192));
  nanp03aa1n06x5               g097(.a(new_n149), .b(new_n192), .c(new_n188), .o1(new_n193));
  nand23aa1n09x5               g098(.a(new_n187), .b(new_n193), .c(new_n186), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n125), .d(new_n184), .o1(new_n196));
  nano32aa1n02x4               g101(.a(new_n195), .b(new_n187), .c(new_n193), .d(new_n186), .out0(new_n197));
  aobi12aa1n02x5               g102(.a(new_n196), .b(new_n197), .c(new_n185), .out0(\s[17] ));
  inv000aa1d42x5               g103(.a(\a[17] ), .o1(new_n199));
  nanb02aa1d24x5               g104(.a(\b[16] ), .b(new_n199), .out0(new_n200));
  nor042aa1n04x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand42aa1n03x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  norb02aa1n06x4               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nano22aa1n02x4               g108(.a(new_n201), .b(new_n200), .c(new_n202), .out0(new_n204));
  nanp02aa1n02x5               g109(.a(new_n196), .b(new_n204), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n205), .b(new_n203), .c(new_n200), .d(new_n196), .o1(\s[18] ));
  and002aa1n02x5               g111(.a(new_n195), .b(new_n203), .o(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n194), .c(new_n125), .d(new_n184), .o1(new_n208));
  oaoi03aa1n12x5               g113(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  nor042aa1d18x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanp02aa1n12x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  norb02aa1n06x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n208), .c(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g120(.a(new_n211), .o1(new_n216));
  aob012aa1n03x5               g121(.a(new_n213), .b(new_n208), .c(new_n210), .out0(new_n217));
  nor002aa1n16x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand42aa1d28x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n06x4               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  norb03aa1n02x5               g125(.a(new_n219), .b(new_n211), .c(new_n218), .out0(new_n221));
  nanp02aa1n02x5               g126(.a(new_n217), .b(new_n221), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n222), .b(new_n220), .c(new_n217), .d(new_n216), .o1(\s[20] ));
  nano23aa1n06x5               g128(.a(new_n211), .b(new_n218), .c(new_n219), .d(new_n212), .out0(new_n224));
  nand23aa1n06x5               g129(.a(new_n224), .b(new_n195), .c(new_n203), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n194), .c(new_n125), .d(new_n184), .o1(new_n227));
  aoi112aa1n03x5               g132(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n228));
  oai112aa1n06x5               g133(.a(new_n213), .b(new_n220), .c(new_n228), .d(new_n201), .o1(new_n229));
  aoi012aa1n12x5               g134(.a(new_n218), .b(new_n211), .c(new_n219), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(new_n229), .b(new_n230), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  aob012aa1n02x5               g138(.a(new_n233), .b(new_n227), .c(new_n232), .out0(new_n234));
  inv000aa1n02x5               g139(.a(new_n230), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n235), .b(new_n233), .c(new_n224), .d(new_n209), .o1(new_n236));
  aobi12aa1n02x5               g141(.a(new_n234), .b(new_n236), .c(new_n227), .out0(\s[21] ));
  norp02aa1n02x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[21] ), .b(\a[22] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  xnrc02aa1n02x5               g146(.a(\b[20] ), .b(\a[21] ), .out0(new_n242));
  oai022aa1n02x5               g147(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(\a[22] ), .c(\b[21] ), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n242), .c(new_n227), .d(new_n232), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n241), .c(new_n234), .d(new_n239), .o1(\s[22] ));
  nanb02aa1n12x5               g151(.a(new_n240), .b(new_n233), .out0(new_n247));
  nano32aa1n02x4               g152(.a(new_n247), .b(new_n224), .c(new_n203), .d(new_n195), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n194), .c(new_n125), .d(new_n184), .o1(new_n249));
  aob012aa1n06x5               g154(.a(new_n243), .b(\b[21] ), .c(\a[22] ), .out0(new_n250));
  aoai13aa1n12x5               g155(.a(new_n250), .b(new_n247), .c(new_n229), .d(new_n230), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  xorc02aa1n12x5               g157(.a(\a[23] ), .b(\b[22] ), .out0(new_n253));
  aob012aa1n03x5               g158(.a(new_n253), .b(new_n249), .c(new_n252), .out0(new_n254));
  nor042aa1n02x5               g159(.a(new_n240), .b(new_n242), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n235), .c(new_n224), .d(new_n209), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n253), .o1(new_n257));
  and003aa1n02x5               g162(.a(new_n256), .b(new_n257), .c(new_n250), .o(new_n258));
  aobi12aa1n02x7               g163(.a(new_n254), .b(new_n258), .c(new_n249), .out0(\s[23] ));
  nor002aa1d32x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  inv040aa1n12x5               g165(.a(new_n260), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .out0(new_n262));
  oai022aa1n02x5               g167(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n263));
  aoi012aa1n02x5               g168(.a(new_n263), .b(\a[24] ), .c(\b[23] ), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n264), .b(new_n257), .c(new_n249), .d(new_n252), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n265), .b(new_n262), .c(new_n254), .d(new_n261), .o1(\s[24] ));
  tech160nm_finand02aa1n03p5x5 g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  tech160nm_fixnrc02aa1n05x5   g172(.a(\b[23] ), .b(\a[24] ), .out0(new_n268));
  nano22aa1n12x5               g173(.a(new_n268), .b(new_n261), .c(new_n267), .out0(new_n269));
  nano22aa1n03x7               g174(.a(new_n225), .b(new_n255), .c(new_n269), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n194), .c(new_n125), .d(new_n184), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n269), .o1(new_n272));
  oaoi03aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n261), .o1(new_n273));
  inv000aa1n02x5               g178(.a(new_n273), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n272), .c(new_n256), .d(new_n250), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n275), .o1(new_n276));
  tech160nm_fixorc02aa1n03p5x5 g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  aob012aa1n06x5               g182(.a(new_n277), .b(new_n271), .c(new_n276), .out0(new_n278));
  aoi112aa1n02x5               g183(.a(new_n277), .b(new_n273), .c(new_n251), .d(new_n269), .o1(new_n279));
  aobi12aa1n02x5               g184(.a(new_n278), .b(new_n279), .c(new_n271), .out0(\s[25] ));
  norp02aa1n02x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  tech160nm_fixorc02aa1n02p5x5 g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  nanp02aa1n02x5               g188(.a(\b[25] ), .b(\a[26] ), .o1(new_n284));
  oai022aa1n02x5               g189(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n284), .b(new_n285), .out0(new_n286));
  nanp02aa1n03x5               g191(.a(new_n278), .b(new_n286), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n278), .d(new_n282), .o1(\s[26] ));
  and002aa1n02x7               g193(.a(new_n283), .b(new_n277), .o(new_n289));
  nano32aa1n06x5               g194(.a(new_n225), .b(new_n289), .c(new_n255), .d(new_n269), .out0(new_n290));
  aoai13aa1n12x5               g195(.a(new_n290), .b(new_n194), .c(new_n125), .d(new_n184), .o1(new_n291));
  aoai13aa1n09x5               g196(.a(new_n289), .b(new_n273), .c(new_n251), .d(new_n269), .o1(new_n292));
  nanp02aa1n02x5               g197(.a(new_n285), .b(new_n284), .o1(new_n293));
  nanp03aa1d12x5               g198(.a(new_n291), .b(new_n292), .c(new_n293), .o1(new_n294));
  xorc02aa1n12x5               g199(.a(\a[27] ), .b(\b[26] ), .out0(new_n295));
  aoi122aa1n02x5               g200(.a(new_n295), .b(new_n284), .c(new_n285), .d(new_n275), .e(new_n289), .o1(new_n296));
  aoi022aa1n02x5               g201(.a(new_n296), .b(new_n291), .c(new_n294), .d(new_n295), .o1(\s[27] ));
  nor002aa1n12x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  nor002aa1n02x5               g203(.a(\b[27] ), .b(\a[28] ), .o1(new_n299));
  and002aa1n12x5               g204(.a(\b[27] ), .b(\a[28] ), .o(new_n300));
  norp02aa1n02x5               g205(.a(new_n300), .b(new_n299), .o1(new_n301));
  inv030aa1n03x5               g206(.a(new_n301), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n294), .d(new_n295), .o1(new_n303));
  aoi022aa1n03x5               g208(.a(new_n275), .b(new_n289), .c(new_n284), .d(new_n285), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n295), .o1(new_n305));
  norp03aa1n02x5               g210(.a(new_n300), .b(new_n299), .c(new_n298), .o1(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n305), .c(new_n304), .d(new_n291), .o1(new_n307));
  nanp02aa1n03x5               g212(.a(new_n303), .b(new_n307), .o1(\s[28] ));
  norb02aa1n02x5               g213(.a(new_n295), .b(new_n302), .out0(new_n309));
  nand02aa1n02x5               g214(.a(new_n294), .b(new_n309), .o1(new_n310));
  inv000aa1n02x5               g215(.a(new_n309), .o1(new_n311));
  aoib12aa1n06x5               g216(.a(new_n299), .b(new_n298), .c(new_n300), .out0(new_n312));
  aoai13aa1n02x7               g217(.a(new_n312), .b(new_n311), .c(new_n304), .d(new_n291), .o1(new_n313));
  xorc02aa1n03x5               g218(.a(\a[29] ), .b(\b[28] ), .out0(new_n314));
  norb02aa1n02x5               g219(.a(new_n312), .b(new_n314), .out0(new_n315));
  aoi022aa1n03x5               g220(.a(new_n313), .b(new_n314), .c(new_n310), .d(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  inv000aa1d42x5               g222(.a(new_n314), .o1(new_n318));
  nona32aa1n03x5               g223(.a(new_n294), .b(new_n318), .c(new_n302), .d(new_n305), .out0(new_n319));
  nanp03aa1n02x5               g224(.a(new_n314), .b(new_n295), .c(new_n301), .o1(new_n320));
  tech160nm_fioaoi03aa1n04x5   g225(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n321), .o1(new_n322));
  aoai13aa1n02x7               g227(.a(new_n322), .b(new_n320), .c(new_n304), .d(new_n291), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .out0(new_n324));
  norp02aa1n02x5               g229(.a(\b[28] ), .b(\a[29] ), .o1(new_n325));
  aoi012aa1n02x5               g230(.a(new_n312), .b(\a[29] ), .c(\b[28] ), .o1(new_n326));
  norp03aa1n02x5               g231(.a(new_n326), .b(new_n324), .c(new_n325), .o1(new_n327));
  aoi022aa1n03x5               g232(.a(new_n323), .b(new_n324), .c(new_n319), .d(new_n327), .o1(\s[30] ));
  inv000aa1n02x5               g233(.a(new_n324), .o1(new_n329));
  nona32aa1n03x5               g234(.a(new_n294), .b(new_n329), .c(new_n318), .d(new_n311), .out0(new_n330));
  xorc02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  inv000aa1d42x5               g236(.a(\a[30] ), .o1(new_n332));
  inv000aa1d42x5               g237(.a(\b[29] ), .o1(new_n333));
  oabi12aa1n02x5               g238(.a(new_n331), .b(\a[30] ), .c(\b[29] ), .out0(new_n334));
  oaoi13aa1n04x5               g239(.a(new_n334), .b(new_n321), .c(new_n332), .d(new_n333), .o1(new_n335));
  nona22aa1n02x4               g240(.a(new_n309), .b(new_n318), .c(new_n329), .out0(new_n336));
  oaoi03aa1n02x5               g241(.a(new_n332), .b(new_n333), .c(new_n321), .o1(new_n337));
  aoai13aa1n02x7               g242(.a(new_n337), .b(new_n336), .c(new_n304), .d(new_n291), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n338), .b(new_n331), .c(new_n330), .d(new_n335), .o1(\s[31] ));
  xorc02aa1n02x5               g244(.a(\a[3] ), .b(\b[2] ), .out0(new_n340));
  oao003aa1n02x5               g245(.a(new_n99), .b(new_n104), .c(new_n100), .carry(new_n341));
  oa0012aa1n02x5               g246(.a(new_n106), .b(new_n340), .c(new_n341), .o(\s[3] ));
  xorc02aa1n02x5               g247(.a(\a[4] ), .b(\b[3] ), .out0(new_n343));
  oab012aa1n02x4               g248(.a(new_n343), .b(\a[3] ), .c(\b[2] ), .out0(new_n344));
  aoi022aa1n02x5               g249(.a(new_n156), .b(new_n343), .c(new_n106), .d(new_n344), .o1(\s[4] ));
  nanp02aa1n02x5               g250(.a(\b[3] ), .b(\a[4] ), .o1(new_n346));
  xorc02aa1n02x5               g251(.a(\a[5] ), .b(\b[4] ), .out0(new_n347));
  xobna2aa1n03x5               g252(.a(new_n347), .b(new_n156), .c(new_n346), .out0(\s[5] ));
  inv000aa1d42x5               g253(.a(new_n108), .o1(new_n349));
  nand23aa1n03x5               g254(.a(new_n156), .b(new_n346), .c(new_n347), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n116), .b(new_n118), .out0(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n350), .c(new_n349), .out0(\s[6] ));
  inv000aa1d42x5               g257(.a(new_n118), .o1(new_n353));
  aob012aa1n02x5               g258(.a(new_n351), .b(new_n350), .c(new_n349), .out0(new_n354));
  norb02aa1n02x5               g259(.a(new_n111), .b(new_n109), .out0(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n355), .b(new_n354), .c(new_n353), .out0(\s[7] ));
  aobi12aa1n06x5               g261(.a(new_n355), .b(new_n354), .c(new_n353), .out0(new_n357));
  xnrc02aa1n02x5               g262(.a(\b[7] ), .b(\a[8] ), .out0(new_n358));
  oai012aa1n02x5               g263(.a(new_n358), .b(new_n357), .c(new_n109), .o1(new_n359));
  norp02aa1n02x5               g264(.a(new_n358), .b(new_n109), .o1(new_n360));
  oaib12aa1n02x5               g265(.a(new_n359), .b(new_n357), .c(new_n360), .out0(\s[8] ));
  norb03aa1n02x5               g266(.a(new_n123), .b(new_n121), .c(new_n126), .out0(new_n362));
  aoi022aa1n02x5               g267(.a(new_n125), .b(new_n126), .c(new_n157), .d(new_n362), .o1(\s[9] ));
endmodule


