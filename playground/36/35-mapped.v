// Benchmark "adder" written by ABC on Thu Jul 18 06:43:25 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n341, new_n342, new_n345, new_n346, new_n348,
    new_n349, new_n350, new_n352, new_n353, new_n355, new_n356;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  orn002aa1n12x5               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  nand42aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n24x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n06x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n101));
  nand42aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n06x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  nor042aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n06x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nand23aa1n09x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  oaih12aa1n06x5               g013(.a(new_n102), .b(new_n105), .c(new_n103), .o1(new_n109));
  nor002aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1n08x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand42aa1n10x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor042aa1d18x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1d15x5               g018(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n114));
  tech160nm_fixorc02aa1n03p5x5 g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  tech160nm_fixorc02aa1n05x5   g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  nand23aa1n03x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nor042aa1d18x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  inv040aa1n08x5               g023(.a(new_n118), .o1(new_n119));
  oaoi03aa1n12x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  inv000aa1n02x5               g025(.a(new_n113), .o1(new_n121));
  tech160nm_fioaoi03aa1n04x5   g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  aoi012aa1n12x5               g027(.a(new_n122), .b(new_n114), .c(new_n120), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n123), .b(new_n117), .c(new_n108), .d(new_n109), .o1(new_n124));
  xnrc02aa1n12x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  nanp02aa1n09x5               g031(.a(new_n124), .b(new_n126), .o1(new_n127));
  tech160nm_fixorc02aa1n05x5   g032(.a(\a[10] ), .b(\b[9] ), .out0(new_n128));
  inv000aa1d42x5               g033(.a(\a[10] ), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\b[9] ), .o1(new_n130));
  oai022aa1d24x5               g035(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  oai112aa1n02x5               g037(.a(new_n127), .b(new_n132), .c(new_n130), .d(new_n129), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n128), .c(new_n97), .d(new_n127), .o1(\s[10] ));
  nanp02aa1n02x5               g039(.a(new_n127), .b(new_n132), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  orn002aa1n12x5               g041(.a(\a[11] ), .b(\b[10] ), .o(new_n137));
  oai112aa1n02x5               g042(.a(new_n137), .b(new_n136), .c(new_n130), .d(new_n129), .o1(new_n138));
  nanp02aa1n06x5               g043(.a(new_n137), .b(new_n136), .o1(new_n139));
  oaib12aa1n02x5               g044(.a(new_n135), .b(new_n130), .c(\a[10] ), .out0(new_n140));
  aboi22aa1n03x5               g045(.a(new_n138), .b(new_n135), .c(new_n140), .d(new_n139), .out0(\s[11] ));
  nanb02aa1n02x5               g046(.a(new_n138), .b(new_n135), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n137), .b(new_n138), .c(new_n127), .d(new_n132), .o1(new_n143));
  xorc02aa1n06x5               g048(.a(\a[12] ), .b(\b[11] ), .out0(new_n144));
  norb02aa1n02x5               g049(.a(new_n137), .b(new_n144), .out0(new_n145));
  aoi022aa1n02x5               g050(.a(new_n142), .b(new_n145), .c(new_n143), .d(new_n144), .o1(\s[12] ));
  nona23aa1d18x5               g051(.a(new_n144), .b(new_n128), .c(new_n125), .d(new_n139), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  oai112aa1n03x5               g054(.a(new_n131), .b(new_n136), .c(new_n130), .d(new_n129), .o1(new_n150));
  oai112aa1n03x5               g055(.a(new_n150), .b(new_n137), .c(\b[11] ), .d(\a[12] ), .o1(new_n151));
  and002aa1n02x5               g056(.a(new_n151), .b(new_n149), .o(new_n152));
  xorc02aa1n02x5               g057(.a(\a[13] ), .b(\b[12] ), .out0(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n152), .c(new_n124), .d(new_n148), .o1(new_n154));
  aoi112aa1n02x5               g059(.a(new_n153), .b(new_n152), .c(new_n124), .d(new_n148), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[12] ), .o1(new_n158));
  nand42aa1n03x5               g063(.a(new_n158), .b(new_n157), .o1(new_n159));
  xorc02aa1n02x5               g064(.a(\a[14] ), .b(\b[13] ), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n154), .c(new_n159), .out0(\s[14] ));
  inv000aa1d42x5               g066(.a(\a[14] ), .o1(new_n162));
  xroi22aa1d04x5               g067(.a(new_n157), .b(\b[12] ), .c(new_n162), .d(\b[13] ), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n152), .c(new_n124), .d(new_n148), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[13] ), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n166));
  aoi012aa1n02x5               g071(.a(new_n166), .b(new_n162), .c(new_n165), .o1(new_n167));
  xorc02aa1n02x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n164), .c(new_n167), .out0(\s[15] ));
  nanp02aa1n02x5               g074(.a(new_n164), .b(new_n167), .o1(new_n170));
  nand42aa1n02x5               g075(.a(new_n170), .b(new_n168), .o1(new_n171));
  inv040aa1d32x5               g076(.a(\a[15] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\b[14] ), .o1(new_n173));
  nand42aa1n02x5               g078(.a(new_n173), .b(new_n172), .o1(new_n174));
  nand42aa1n08x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n174), .b(new_n176), .c(new_n164), .d(new_n167), .o1(new_n177));
  nor042aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  and002aa1n02x7               g083(.a(\b[15] ), .b(\a[16] ), .o(new_n179));
  norp02aa1n02x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n174), .b(new_n180), .out0(new_n181));
  aoi022aa1n03x5               g086(.a(new_n177), .b(new_n180), .c(new_n171), .d(new_n181), .o1(\s[16] ));
  nano23aa1n02x4               g087(.a(new_n179), .b(new_n178), .c(new_n174), .d(new_n175), .out0(new_n183));
  nano22aa1d15x5               g088(.a(new_n147), .b(new_n163), .c(new_n183), .out0(new_n184));
  nanp02aa1n06x5               g089(.a(new_n124), .b(new_n184), .o1(new_n185));
  aoi112aa1n02x7               g090(.a(new_n179), .b(new_n178), .c(\a[15] ), .d(\b[14] ), .o1(new_n186));
  oai022aa1n02x5               g091(.a(new_n157), .b(new_n158), .c(\b[13] ), .d(\a[14] ), .o1(new_n187));
  oai022aa1n02x5               g092(.a(new_n162), .b(new_n165), .c(\b[14] ), .d(\a[15] ), .o1(new_n188));
  nano23aa1n02x5               g093(.a(new_n188), .b(new_n187), .c(new_n159), .d(new_n149), .out0(new_n189));
  aoi012aa1n02x5               g094(.a(new_n178), .b(new_n172), .c(new_n173), .o1(new_n190));
  oaoi13aa1n02x5               g095(.a(new_n179), .b(new_n190), .c(new_n167), .d(new_n176), .o1(new_n191));
  aoi013aa1n02x4               g096(.a(new_n191), .b(new_n151), .c(new_n186), .d(new_n189), .o1(new_n192));
  nand02aa1d06x5               g097(.a(new_n185), .b(new_n192), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  aoi113aa1n02x5               g099(.a(new_n191), .b(new_n194), .c(new_n151), .d(new_n186), .e(new_n189), .o1(new_n195));
  aoi022aa1n02x5               g100(.a(new_n193), .b(new_n194), .c(new_n185), .d(new_n195), .o1(\s[17] ));
  inv000aa1d42x5               g101(.a(\a[17] ), .o1(new_n197));
  nanb02aa1n02x5               g102(.a(\b[16] ), .b(new_n197), .out0(new_n198));
  nanp03aa1n02x5               g103(.a(new_n151), .b(new_n186), .c(new_n189), .o1(new_n199));
  oaoi03aa1n02x5               g104(.a(\a[14] ), .b(\b[13] ), .c(new_n159), .o1(new_n200));
  aobi12aa1n02x5               g105(.a(new_n190), .b(new_n200), .c(new_n175), .out0(new_n201));
  tech160nm_fioai012aa1n05x5   g106(.a(new_n199), .b(new_n201), .c(new_n179), .o1(new_n202));
  aoai13aa1n03x5               g107(.a(new_n194), .b(new_n202), .c(new_n124), .d(new_n184), .o1(new_n203));
  nor042aa1n04x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand02aa1n06x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  norb02aa1n06x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n203), .c(new_n198), .out0(\s[18] ));
  and002aa1n06x5               g112(.a(new_n194), .b(new_n206), .o(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n202), .c(new_n124), .d(new_n184), .o1(new_n209));
  nor042aa1n02x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  aoi012aa1n02x5               g115(.a(new_n204), .b(new_n210), .c(new_n205), .o1(new_n211));
  xorc02aa1n12x5               g116(.a(\a[19] ), .b(\b[18] ), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand02aa1n02x5               g119(.a(new_n209), .b(new_n211), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n215), .b(new_n212), .o1(new_n216));
  xorc02aa1n12x5               g121(.a(\a[20] ), .b(\b[19] ), .out0(new_n217));
  inv000aa1d42x5               g122(.a(\a[19] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\b[18] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(new_n219), .b(new_n218), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n217), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n212), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n220), .b(new_n222), .c(new_n209), .d(new_n211), .o1(new_n223));
  aoi022aa1n03x5               g128(.a(new_n223), .b(new_n217), .c(new_n216), .d(new_n221), .o1(\s[20] ));
  nand23aa1n06x5               g129(.a(new_n208), .b(new_n212), .c(new_n217), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n202), .c(new_n124), .d(new_n184), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[18] ), .b(\a[19] ), .o1(new_n228));
  aoai13aa1n04x5               g133(.a(new_n228), .b(new_n204), .c(new_n210), .d(new_n205), .o1(new_n229));
  oa0022aa1n02x5               g134(.a(\a[20] ), .b(\b[19] ), .c(\a[19] ), .d(\b[18] ), .o(new_n230));
  aoi022aa1n06x5               g135(.a(new_n229), .b(new_n230), .c(\b[19] ), .d(\a[20] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  xnbna2aa1n03x5               g138(.a(new_n233), .b(new_n227), .c(new_n232), .out0(\s[21] ));
  aoai13aa1n03x5               g139(.a(new_n233), .b(new_n231), .c(new_n193), .d(new_n226), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[22] ), .b(\b[21] ), .out0(new_n236));
  nor042aa1n03x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norp02aa1n02x5               g142(.a(new_n236), .b(new_n237), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n237), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n233), .o1(new_n240));
  aoai13aa1n02x5               g145(.a(new_n239), .b(new_n240), .c(new_n227), .d(new_n232), .o1(new_n241));
  aoi022aa1n03x5               g146(.a(new_n241), .b(new_n236), .c(new_n235), .d(new_n238), .o1(\s[22] ));
  and002aa1n02x7               g147(.a(new_n236), .b(new_n233), .o(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n225), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n202), .c(new_n124), .d(new_n184), .o1(new_n245));
  nand22aa1n03x5               g150(.a(new_n229), .b(new_n230), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[19] ), .b(\a[20] ), .o1(new_n247));
  oai012aa1n02x5               g152(.a(new_n247), .b(\b[20] ), .c(\a[21] ), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  oai012aa1n02x5               g154(.a(new_n249), .b(\b[21] ), .c(\a[22] ), .o1(new_n250));
  aoi112aa1n03x5               g155(.a(new_n250), .b(new_n248), .c(\a[22] ), .d(\b[21] ), .o1(new_n251));
  oaoi03aa1n12x5               g156(.a(\a[22] ), .b(\b[21] ), .c(new_n239), .o1(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n246), .c(new_n251), .o1(new_n253));
  nanp02aa1n03x5               g158(.a(new_n245), .b(new_n253), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  aoi112aa1n02x5               g160(.a(new_n255), .b(new_n252), .c(new_n246), .d(new_n251), .o1(new_n256));
  aoi022aa1n02x5               g161(.a(new_n254), .b(new_n255), .c(new_n245), .d(new_n256), .o1(\s[23] ));
  nanp02aa1n03x5               g162(.a(new_n254), .b(new_n255), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .out0(new_n259));
  nor042aa1n03x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  norp02aa1n02x5               g165(.a(new_n259), .b(new_n260), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n260), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n255), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n262), .b(new_n263), .c(new_n245), .d(new_n253), .o1(new_n264));
  aoi022aa1n03x5               g169(.a(new_n264), .b(new_n259), .c(new_n258), .d(new_n261), .o1(\s[24] ));
  nanp02aa1n02x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[23] ), .b(\a[24] ), .out0(new_n267));
  nano22aa1n06x5               g172(.a(new_n267), .b(new_n262), .c(new_n266), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n225), .b(new_n243), .c(new_n268), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n202), .c(new_n124), .d(new_n184), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n268), .b(new_n252), .c(new_n246), .d(new_n251), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .c(new_n262), .carry(new_n272));
  nand42aa1n04x5               g177(.a(new_n271), .b(new_n272), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  nanp02aa1n03x5               g179(.a(new_n270), .b(new_n274), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[25] ), .b(\b[24] ), .out0(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  and003aa1n02x5               g182(.a(new_n271), .b(new_n277), .c(new_n272), .o(new_n278));
  aoi022aa1n02x5               g183(.a(new_n275), .b(new_n276), .c(new_n270), .d(new_n278), .o1(\s[25] ));
  nanp02aa1n03x5               g184(.a(new_n275), .b(new_n276), .o1(new_n280));
  tech160nm_fixorc02aa1n02p5x5 g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n281), .b(new_n282), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n282), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n277), .c(new_n270), .d(new_n274), .o1(new_n285));
  aoi022aa1n03x5               g190(.a(new_n285), .b(new_n281), .c(new_n280), .d(new_n283), .o1(\s[26] ));
  and002aa1n12x5               g191(.a(new_n281), .b(new_n276), .o(new_n287));
  nano32aa1n03x7               g192(.a(new_n225), .b(new_n287), .c(new_n243), .d(new_n268), .out0(new_n288));
  aoai13aa1n12x5               g193(.a(new_n288), .b(new_n202), .c(new_n124), .d(new_n184), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(\b[25] ), .b(\a[26] ), .o1(new_n290));
  oai022aa1n02x5               g195(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n291));
  aoi022aa1n09x5               g196(.a(new_n273), .b(new_n287), .c(new_n290), .d(new_n291), .o1(new_n292));
  nanp02aa1n03x5               g197(.a(new_n292), .b(new_n289), .o1(new_n293));
  xorc02aa1n12x5               g198(.a(\a[27] ), .b(\b[26] ), .out0(new_n294));
  aoi122aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n291), .d(new_n273), .e(new_n287), .o1(new_n295));
  aoi022aa1n02x5               g200(.a(new_n293), .b(new_n294), .c(new_n295), .d(new_n289), .o1(\s[27] ));
  inv000aa1d42x5               g201(.a(new_n287), .o1(new_n297));
  nanp02aa1n02x5               g202(.a(new_n291), .b(new_n290), .o1(new_n298));
  aoai13aa1n02x5               g203(.a(new_n298), .b(new_n297), .c(new_n271), .d(new_n272), .o1(new_n299));
  aoai13aa1n02x7               g204(.a(new_n294), .b(new_n299), .c(new_n193), .d(new_n288), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[28] ), .b(\b[27] ), .out0(new_n301));
  nor042aa1d18x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n301), .b(new_n302), .o1(new_n303));
  inv000aa1n06x5               g208(.a(new_n302), .o1(new_n304));
  inv000aa1n02x5               g209(.a(new_n294), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n304), .b(new_n305), .c(new_n292), .d(new_n289), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n306), .b(new_n301), .c(new_n300), .d(new_n303), .o1(\s[28] ));
  and002aa1n02x5               g212(.a(new_n301), .b(new_n294), .o(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n299), .c(new_n193), .d(new_n288), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n03x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n310), .c(new_n292), .d(new_n289), .o1(new_n312));
  tech160nm_fixorc02aa1n03p5x5 g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g221(.a(new_n305), .b(new_n301), .c(new_n313), .out0(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n299), .c(new_n193), .d(new_n288), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  tech160nm_fioaoi03aa1n02p5x5 g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n320), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n319), .c(new_n292), .d(new_n289), .o1(new_n322));
  tech160nm_fixorc02aa1n03p5x5 g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  and002aa1n02x5               g228(.a(\b[28] ), .b(\a[29] ), .o(new_n324));
  oabi12aa1n02x5               g229(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .out0(new_n325));
  oab012aa1n02x4               g230(.a(new_n325), .b(new_n311), .c(new_n324), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n322), .b(new_n323), .c(new_n318), .d(new_n326), .o1(\s[30] ));
  nano32aa1n02x5               g232(.a(new_n305), .b(new_n323), .c(new_n301), .d(new_n313), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n299), .c(new_n193), .d(new_n288), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[31] ), .b(\b[30] ), .out0(new_n330));
  inv000aa1d42x5               g235(.a(\a[30] ), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[29] ), .o1(new_n332));
  oabi12aa1n02x5               g237(.a(new_n330), .b(\a[30] ), .c(\b[29] ), .out0(new_n333));
  oaoi13aa1n03x5               g238(.a(new_n333), .b(new_n320), .c(new_n331), .d(new_n332), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n328), .o1(new_n335));
  oaoi03aa1n02x5               g240(.a(new_n331), .b(new_n332), .c(new_n320), .o1(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n335), .c(new_n292), .d(new_n289), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n330), .c(new_n329), .d(new_n334), .o1(\s[31] ));
  nanp03aa1n02x5               g243(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n107), .b(new_n339), .c(new_n98), .out0(\s[3] ));
  aoai13aa1n02x5               g245(.a(new_n104), .b(new_n105), .c(new_n101), .d(new_n106), .o1(new_n341));
  aoi112aa1n02x5               g246(.a(new_n105), .b(new_n104), .c(new_n101), .d(new_n106), .o1(new_n342));
  norb02aa1n02x5               g247(.a(new_n341), .b(new_n342), .out0(\s[4] ));
  xnbna2aa1n03x5               g248(.a(new_n116), .b(new_n108), .c(new_n109), .out0(\s[5] ));
  nanp02aa1n02x5               g249(.a(new_n108), .b(new_n109), .o1(new_n345));
  nanp02aa1n02x5               g250(.a(new_n345), .b(new_n116), .o1(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n115), .b(new_n346), .c(new_n119), .out0(\s[6] ));
  norb02aa1n02x5               g252(.a(new_n112), .b(new_n113), .out0(new_n348));
  inv000aa1d42x5               g253(.a(new_n120), .o1(new_n349));
  nanp03aa1n02x5               g254(.a(new_n345), .b(new_n115), .c(new_n116), .o1(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n348), .b(new_n350), .c(new_n349), .out0(\s[7] ));
  norb02aa1n02x5               g256(.a(new_n111), .b(new_n110), .out0(new_n352));
  aob012aa1n02x5               g257(.a(new_n348), .b(new_n350), .c(new_n349), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n352), .b(new_n353), .c(new_n121), .out0(\s[8] ));
  nanb02aa1n02x5               g259(.a(new_n117), .b(new_n345), .out0(new_n355));
  aoi112aa1n02x5               g260(.a(new_n126), .b(new_n122), .c(new_n114), .d(new_n120), .o1(new_n356));
  aoi022aa1n02x5               g261(.a(new_n124), .b(new_n126), .c(new_n355), .d(new_n356), .o1(\s[9] ));
endmodule


