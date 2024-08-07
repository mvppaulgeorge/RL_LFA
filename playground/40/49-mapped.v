// Benchmark "adder" written by ABC on Thu Jul 18 08:55:12 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n328, new_n331, new_n333, new_n334,
    new_n335, new_n336, new_n337, new_n338, new_n340, new_n341;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d16x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1d28x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1d18x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  tech160nm_fixnrc02aa1n04x5   g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  tech160nm_fixnrc02aa1n04x5   g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[3] ), .o1(new_n105));
  nanb02aa1n12x5               g010(.a(\b[2] ), .b(new_n105), .out0(new_n106));
  oao003aa1n03x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .carry(new_n107));
  oai013aa1n09x5               g012(.a(new_n107), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n108));
  nor002aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand02aa1d24x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand02aa1n10x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n06x5               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  inv000aa1n02x5               g018(.a(new_n113), .o1(new_n114));
  xorc02aa1n06x5               g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  nano22aa1n03x7               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  orn002aa1n24x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n12x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  tech160nm_fiao0012aa1n02p5x5 g024(.a(new_n109), .b(new_n111), .c(new_n110), .o(new_n120));
  aoi012aa1n06x5               g025(.a(new_n120), .b(new_n113), .c(new_n119), .o1(new_n121));
  inv030aa1n03x5               g026(.a(new_n121), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n122), .c(new_n117), .d(new_n108), .o1(new_n124));
  nor022aa1n16x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n12x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanb02aa1n12x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  xobna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  xorc02aa1n12x5               g033(.a(\a[11] ), .b(\b[10] ), .out0(new_n129));
  nona22aa1n03x5               g034(.a(new_n124), .b(new_n125), .c(new_n97), .out0(new_n130));
  xobna2aa1n03x5               g035(.a(new_n129), .b(new_n130), .c(new_n126), .out0(\s[11] ));
  orn002aa1n02x5               g036(.a(\a[11] ), .b(\b[10] ), .o(new_n132));
  and002aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o(new_n133));
  nano22aa1n02x4               g038(.a(new_n133), .b(new_n132), .c(new_n126), .out0(new_n134));
  aobi12aa1n06x5               g039(.a(new_n132), .b(new_n130), .c(new_n134), .out0(new_n135));
  orn002aa1n12x5               g040(.a(\a[12] ), .b(\b[11] ), .o(new_n136));
  nand02aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n135), .b(new_n137), .c(new_n136), .out0(\s[12] ));
  xorc02aa1n06x5               g043(.a(\a[4] ), .b(\b[3] ), .out0(new_n139));
  xorc02aa1n12x5               g044(.a(\a[3] ), .b(\b[2] ), .out0(new_n140));
  nanb03aa1n12x5               g045(.a(new_n102), .b(new_n140), .c(new_n139), .out0(new_n141));
  nand03aa1n02x5               g046(.a(new_n113), .b(new_n115), .c(new_n116), .o1(new_n142));
  aoai13aa1n06x5               g047(.a(new_n121), .b(new_n142), .c(new_n141), .d(new_n107), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(new_n136), .b(new_n137), .o1(new_n144));
  nona23aa1d18x5               g049(.a(new_n129), .b(new_n123), .c(new_n144), .d(new_n127), .out0(new_n145));
  nanb02aa1n06x5               g050(.a(new_n145), .b(new_n143), .out0(new_n146));
  inv000aa1d42x5               g051(.a(\a[9] ), .o1(new_n147));
  inv000aa1d42x5               g052(.a(\b[8] ), .o1(new_n148));
  aoai13aa1n04x5               g053(.a(new_n126), .b(new_n125), .c(new_n147), .d(new_n148), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n136), .b(new_n133), .c(new_n149), .d(new_n132), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n137), .o1(new_n151));
  nor022aa1n08x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n146), .c(new_n151), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(\a[14] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[12] ), .o1(new_n158));
  nand22aa1n03x5               g063(.a(new_n146), .b(new_n151), .o1(new_n159));
  oaoi03aa1n03x5               g064(.a(new_n157), .b(new_n158), .c(new_n159), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(new_n156), .out0(\s[14] ));
  nor022aa1n16x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand02aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1d18x5               g068(.a(new_n163), .b(new_n153), .c(new_n152), .d(new_n162), .out0(new_n164));
  nanb02aa1n02x5               g069(.a(new_n164), .b(new_n159), .out0(new_n165));
  aoai13aa1n06x5               g070(.a(new_n163), .b(new_n162), .c(new_n157), .d(new_n158), .o1(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n164), .c(new_n146), .d(new_n151), .o1(new_n167));
  nor002aa1n20x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n06x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  oai022aa1n02x5               g075(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n168), .o1(new_n172));
  aoi022aa1n02x5               g077(.a(new_n171), .b(new_n163), .c(new_n172), .d(new_n169), .o1(new_n173));
  aoi022aa1n02x5               g078(.a(new_n173), .b(new_n165), .c(new_n167), .d(new_n170), .o1(\s[15] ));
  nor002aa1n12x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand02aa1d20x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1d27x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n168), .c(new_n167), .d(new_n169), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n167), .b(new_n170), .o1(new_n180));
  nona22aa1n02x4               g085(.a(new_n180), .b(new_n178), .c(new_n168), .out0(new_n181));
  nanp02aa1n03x5               g086(.a(new_n181), .b(new_n179), .o1(\s[16] ));
  nano22aa1d15x5               g087(.a(new_n164), .b(new_n170), .c(new_n177), .out0(new_n183));
  norb02aa1n15x5               g088(.a(new_n183), .b(new_n145), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n122), .c(new_n117), .d(new_n108), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n169), .b(new_n168), .c(new_n171), .d(new_n163), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(\a[16] ), .b(\b[15] ), .c(new_n186), .o1(new_n187));
  aoi013aa1n06x5               g092(.a(new_n187), .b(new_n150), .c(new_n183), .d(new_n137), .o1(new_n188));
  xorc02aa1n12x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n185), .c(new_n188), .out0(\s[17] ));
  inv000aa1d42x5               g095(.a(\a[17] ), .o1(new_n191));
  inv040aa1d28x5               g096(.a(\b[16] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nanp03aa1n02x5               g098(.a(new_n150), .b(new_n183), .c(new_n137), .o1(new_n194));
  nand02aa1n02x5               g099(.a(new_n166), .b(new_n172), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n176), .b(new_n175), .c(new_n195), .d(new_n169), .o1(new_n196));
  nanp02aa1n06x5               g101(.a(new_n194), .b(new_n196), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n189), .b(new_n197), .c(new_n143), .d(new_n184), .o1(new_n198));
  nor022aa1n16x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand42aa1n08x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nanb02aa1n12x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  xobna2aa1n03x5               g106(.a(new_n201), .b(new_n198), .c(new_n193), .out0(\s[18] ));
  norb02aa1n02x5               g107(.a(new_n189), .b(new_n201), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n197), .c(new_n143), .d(new_n184), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n200), .b(new_n199), .c(new_n191), .d(new_n192), .o1(new_n205));
  nor002aa1d32x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nand02aa1n06x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  norb02aa1n15x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n204), .c(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g115(.a(new_n204), .b(new_n205), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand02aa1d28x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n12x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n206), .c(new_n211), .d(new_n208), .o1(new_n215));
  nanp02aa1n06x5               g120(.a(new_n185), .b(new_n188), .o1(new_n216));
  oaoi03aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n193), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n208), .b(new_n217), .c(new_n216), .d(new_n203), .o1(new_n218));
  nona22aa1n03x5               g123(.a(new_n218), .b(new_n214), .c(new_n206), .out0(new_n219));
  nanp02aa1n03x5               g124(.a(new_n215), .b(new_n219), .o1(\s[20] ));
  nona23aa1d18x5               g125(.a(new_n213), .b(new_n207), .c(new_n206), .d(new_n212), .out0(new_n221));
  ao0012aa1n12x5               g126(.a(new_n212), .b(new_n206), .c(new_n213), .o(new_n222));
  oabi12aa1n18x5               g127(.a(new_n222), .b(new_n221), .c(new_n205), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  nona23aa1d16x5               g129(.a(new_n208), .b(new_n189), .c(new_n214), .d(new_n201), .out0(new_n225));
  aoai13aa1n04x5               g130(.a(new_n224), .b(new_n225), .c(new_n185), .d(new_n188), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n225), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n228), .b(new_n216), .c(new_n229), .o1(new_n230));
  aoi022aa1n02x5               g135(.a(new_n230), .b(new_n224), .c(new_n226), .d(new_n228), .o1(\s[21] ));
  nor002aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n232), .c(new_n226), .d(new_n228), .o1(new_n234));
  nand02aa1n02x5               g139(.a(new_n226), .b(new_n228), .o1(new_n235));
  nona22aa1n02x4               g140(.a(new_n235), .b(new_n233), .c(new_n232), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n236), .b(new_n234), .o1(\s[22] ));
  norp02aa1n24x5               g142(.a(new_n233), .b(new_n227), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  nor042aa1n02x5               g144(.a(new_n225), .b(new_n239), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n197), .c(new_n143), .d(new_n184), .o1(new_n241));
  nano23aa1n02x5               g146(.a(new_n206), .b(new_n212), .c(new_n213), .d(new_n207), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n238), .b(new_n222), .c(new_n242), .d(new_n217), .o1(new_n243));
  inv000aa1d42x5               g148(.a(\a[22] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(\b[21] ), .o1(new_n245));
  oao003aa1n02x5               g150(.a(new_n244), .b(new_n245), .c(new_n232), .carry(new_n246));
  inv000aa1n02x5               g151(.a(new_n246), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n243), .b(new_n247), .o1(new_n248));
  nanb02aa1n03x5               g153(.a(new_n248), .b(new_n241), .out0(new_n249));
  xorc02aa1n12x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  aoi112aa1n02x5               g155(.a(new_n250), .b(new_n246), .c(new_n223), .d(new_n238), .o1(new_n251));
  aoi022aa1n02x5               g156(.a(new_n249), .b(new_n250), .c(new_n241), .d(new_n251), .o1(\s[23] ));
  norp02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  tech160nm_fixnrc02aa1n02p5x5 g158(.a(\b[23] ), .b(\a[24] ), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n253), .c(new_n249), .d(new_n250), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n250), .b(new_n248), .c(new_n216), .d(new_n240), .o1(new_n256));
  nona22aa1n03x5               g161(.a(new_n256), .b(new_n254), .c(new_n253), .out0(new_n257));
  nanp02aa1n03x5               g162(.a(new_n255), .b(new_n257), .o1(\s[24] ));
  norb02aa1n03x5               g163(.a(new_n250), .b(new_n254), .out0(new_n259));
  nanb03aa1n02x5               g164(.a(new_n225), .b(new_n259), .c(new_n238), .out0(new_n260));
  inv000aa1n02x5               g165(.a(new_n259), .o1(new_n261));
  orn002aa1n02x5               g166(.a(\a[23] ), .b(\b[22] ), .o(new_n262));
  oao003aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .c(new_n262), .carry(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n261), .c(new_n243), .d(new_n247), .o1(new_n264));
  inv000aa1n03x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n260), .c(new_n185), .d(new_n188), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  xorc02aa1n06x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n271));
  nand42aa1n02x5               g176(.a(new_n266), .b(new_n269), .o1(new_n272));
  nona22aa1n03x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .out0(new_n273));
  nanp02aa1n03x5               g178(.a(new_n273), .b(new_n271), .o1(\s[26] ));
  norb02aa1n02x7               g179(.a(new_n269), .b(new_n270), .out0(new_n275));
  inv000aa1n02x5               g180(.a(new_n275), .o1(new_n276));
  nano23aa1n06x5               g181(.a(new_n276), .b(new_n225), .c(new_n259), .d(new_n238), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n197), .c(new_n143), .d(new_n184), .o1(new_n278));
  nand42aa1n02x5               g183(.a(new_n264), .b(new_n275), .o1(new_n279));
  aoi112aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n280));
  oab012aa1n02x4               g185(.a(new_n280), .b(\a[26] ), .c(\b[25] ), .out0(new_n281));
  nanp03aa1n03x5               g186(.a(new_n278), .b(new_n279), .c(new_n281), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  nano22aa1n02x4               g188(.a(new_n283), .b(new_n279), .c(new_n281), .out0(new_n284));
  aoi022aa1n02x5               g189(.a(new_n284), .b(new_n278), .c(new_n282), .d(new_n283), .o1(\s[27] ));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  norp02aa1n02x5               g191(.a(\b[27] ), .b(\a[28] ), .o1(new_n287));
  nand42aa1n03x5               g192(.a(\b[27] ), .b(\a[28] ), .o1(new_n288));
  nanb02aa1n06x5               g193(.a(new_n287), .b(new_n288), .out0(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n286), .c(new_n282), .d(new_n283), .o1(new_n290));
  inv000aa1n02x5               g195(.a(new_n277), .o1(new_n291));
  aoi012aa1n06x5               g196(.a(new_n291), .b(new_n185), .c(new_n188), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n259), .b(new_n246), .c(new_n223), .d(new_n238), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n281), .b(new_n276), .c(new_n293), .d(new_n263), .o1(new_n294));
  oaih12aa1n02x5               g199(.a(new_n283), .b(new_n294), .c(new_n292), .o1(new_n295));
  nona22aa1n02x4               g200(.a(new_n295), .b(new_n289), .c(new_n286), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n290), .b(new_n296), .o1(\s[28] ));
  norb02aa1n03x5               g202(.a(new_n283), .b(new_n289), .out0(new_n298));
  oaih12aa1n02x5               g203(.a(new_n298), .b(new_n294), .c(new_n292), .o1(new_n299));
  aobi12aa1n06x5               g204(.a(new_n281), .b(new_n264), .c(new_n275), .out0(new_n300));
  inv000aa1d42x5               g205(.a(new_n298), .o1(new_n301));
  oai012aa1n02x5               g206(.a(new_n288), .b(new_n287), .c(new_n286), .o1(new_n302));
  aoai13aa1n02x7               g207(.a(new_n302), .b(new_n301), .c(new_n300), .d(new_n278), .o1(new_n303));
  norp02aa1n02x5               g208(.a(\b[28] ), .b(\a[29] ), .o1(new_n304));
  nand42aa1n03x5               g209(.a(\b[28] ), .b(\a[29] ), .o1(new_n305));
  norb02aa1n02x5               g210(.a(new_n305), .b(new_n304), .out0(new_n306));
  oai022aa1n02x5               g211(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n307));
  aboi22aa1n03x5               g212(.a(new_n304), .b(new_n305), .c(new_n307), .d(new_n288), .out0(new_n308));
  aoi022aa1n03x5               g213(.a(new_n303), .b(new_n306), .c(new_n299), .d(new_n308), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g215(.a(new_n289), .b(new_n283), .c(new_n306), .out0(new_n311));
  oaih12aa1n02x5               g216(.a(new_n311), .b(new_n294), .c(new_n292), .o1(new_n312));
  inv000aa1n02x5               g217(.a(new_n311), .o1(new_n313));
  aoi013aa1n02x4               g218(.a(new_n304), .b(new_n307), .c(new_n288), .d(new_n305), .o1(new_n314));
  aoai13aa1n02x7               g219(.a(new_n314), .b(new_n313), .c(new_n300), .d(new_n278), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  aoi113aa1n02x5               g221(.a(new_n316), .b(new_n304), .c(new_n307), .d(new_n305), .e(new_n288), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[30] ));
  nand03aa1n02x5               g223(.a(new_n298), .b(new_n306), .c(new_n316), .o1(new_n319));
  oabi12aa1n03x5               g224(.a(new_n319), .b(new_n294), .c(new_n292), .out0(new_n320));
  xorc02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n322), .b(new_n319), .c(new_n300), .d(new_n278), .o1(new_n324));
  aoi022aa1n03x5               g229(.a(new_n324), .b(new_n321), .c(new_n320), .d(new_n323), .o1(\s[31] ));
  xorb03aa1n02x5               g230(.a(new_n102), .b(\b[2] ), .c(new_n105), .out0(\s[3] ));
  orn002aa1n02x5               g231(.a(\a[4] ), .b(\b[3] ), .o(new_n327));
  oai112aa1n02x5               g232(.a(new_n106), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n328));
  aobi12aa1n02x5               g233(.a(new_n328), .b(new_n108), .c(new_n327), .out0(\s[4] ));
  xnbna2aa1n03x5               g234(.a(new_n116), .b(new_n141), .c(new_n107), .out0(\s[5] ));
  nanp02aa1n02x5               g235(.a(new_n108), .b(new_n116), .o1(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n115), .b(new_n331), .c(new_n118), .out0(\s[6] ));
  inv000aa1d42x5               g237(.a(\a[6] ), .o1(new_n333));
  inv000aa1d42x5               g238(.a(\b[5] ), .o1(new_n334));
  norb02aa1n02x5               g239(.a(new_n112), .b(new_n111), .out0(new_n335));
  aobi12aa1n06x5               g240(.a(new_n115), .b(new_n331), .c(new_n118), .out0(new_n336));
  aoai13aa1n06x5               g241(.a(new_n335), .b(new_n336), .c(new_n334), .d(new_n333), .o1(new_n337));
  aoi112aa1n02x5               g242(.a(new_n336), .b(new_n335), .c(new_n333), .d(new_n334), .o1(new_n338));
  norb02aa1n02x5               g243(.a(new_n337), .b(new_n338), .out0(\s[7] ));
  norb02aa1n02x5               g244(.a(new_n110), .b(new_n109), .out0(new_n340));
  orn002aa1n02x5               g245(.a(\a[7] ), .b(\b[6] ), .o(new_n341));
  xnbna2aa1n03x5               g246(.a(new_n340), .b(new_n337), .c(new_n341), .out0(\s[8] ));
  xorb03aa1n02x5               g247(.a(new_n143), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


