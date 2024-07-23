// Benchmark "adder" written by ABC on Wed Jul 17 16:54:54 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n324,
    new_n325, new_n328, new_n330, new_n332, new_n333, new_n334, new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  nor042aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor042aa1n09x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1d10x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n03x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  xorc02aa1n12x5               g010(.a(\a[3] ), .b(\b[2] ), .out0(new_n106));
  nanb03aa1n12x5               g011(.a(new_n102), .b(new_n106), .c(new_n105), .out0(new_n107));
  nor002aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  aoi012aa1n06x5               g013(.a(new_n103), .b(new_n108), .c(new_n104), .o1(new_n109));
  xorc02aa1n12x5               g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  inv000aa1d42x5               g015(.a(\b[6] ), .o1(new_n111));
  nanb02aa1d36x5               g016(.a(\a[7] ), .b(new_n111), .out0(new_n112));
  nand02aa1d28x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand22aa1n03x5               g018(.a(new_n112), .b(new_n113), .o1(new_n114));
  nand02aa1d16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb02aa1n12x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  xorc02aa1n12x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nona23aa1d18x5               g023(.a(new_n110), .b(new_n118), .c(new_n117), .d(new_n114), .out0(new_n119));
  norp02aa1n02x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  inv040aa1n20x5               g025(.a(\a[5] ), .o1(new_n121));
  inv040aa1n20x5               g026(.a(\b[4] ), .o1(new_n122));
  aoai13aa1n06x5               g027(.a(new_n115), .b(new_n116), .c(new_n122), .d(new_n121), .o1(new_n123));
  nand02aa1d04x5               g028(.a(new_n123), .b(new_n112), .o1(new_n124));
  aoi022aa1n12x5               g029(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n125));
  aoi012aa1n12x5               g030(.a(new_n120), .b(new_n124), .c(new_n125), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n119), .c(new_n107), .d(new_n109), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  nanp02aa1n02x5               g033(.a(new_n127), .b(new_n128), .o1(new_n129));
  xobna2aa1n03x5               g034(.a(new_n97), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  tech160nm_finand02aa1n05x5   g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor002aa1d32x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n12x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oai022aa1d24x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n134), .b(new_n129), .out0(new_n135));
  aob012aa1n02x5               g040(.a(new_n135), .b(\b[9] ), .c(\a[10] ), .out0(new_n136));
  inv020aa1n06x5               g041(.a(new_n132), .o1(new_n137));
  aoi022aa1d24x5               g042(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n138));
  nanp02aa1n24x5               g043(.a(new_n138), .b(new_n137), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  aoi022aa1n02x5               g045(.a(new_n136), .b(new_n133), .c(new_n135), .d(new_n140), .o1(\s[11] ));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n134), .c(new_n127), .d(new_n128), .o1(new_n142));
  nor042aa1n09x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1d16x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n06x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n137), .out0(\s[12] ));
  nona23aa1d24x5               g051(.a(new_n145), .b(new_n128), .c(new_n97), .d(new_n133), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n127), .b(new_n148), .o1(new_n149));
  nanb03aa1d24x5               g054(.a(new_n143), .b(new_n134), .c(new_n144), .out0(new_n150));
  aoi012aa1n12x5               g055(.a(new_n143), .b(new_n132), .c(new_n144), .o1(new_n151));
  oai012aa1d24x5               g056(.a(new_n151), .b(new_n150), .c(new_n139), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n149), .b(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand02aa1n06x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  tech160nm_fiaoi012aa1n05x5   g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  orn002aa1n02x5               g063(.a(\a[14] ), .b(\b[13] ), .o(new_n159));
  nand02aa1n06x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n158), .b(new_n160), .c(new_n159), .out0(\s[14] ));
  nor002aa1d32x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1d24x5               g067(.a(new_n160), .b(new_n157), .c(new_n156), .d(new_n162), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n03x5               g069(.a(new_n164), .b(new_n152), .c(new_n127), .d(new_n148), .o1(new_n165));
  tech160nm_fioai012aa1n03p5x5 g070(.a(new_n160), .b(new_n162), .c(new_n156), .o1(new_n166));
  nor022aa1n16x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand02aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n18x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n165), .c(new_n166), .out0(\s[15] ));
  nand42aa1n02x5               g076(.a(new_n165), .b(new_n166), .o1(new_n172));
  nor042aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n09x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n167), .c(new_n172), .d(new_n168), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(new_n172), .b(new_n170), .o1(new_n177));
  nona22aa1n02x4               g082(.a(new_n177), .b(new_n175), .c(new_n167), .out0(new_n178));
  nanp02aa1n03x5               g083(.a(new_n178), .b(new_n176), .o1(\s[16] ));
  nor043aa1d12x5               g084(.a(new_n163), .b(new_n169), .c(new_n175), .o1(new_n180));
  norb02aa1d21x5               g085(.a(new_n180), .b(new_n147), .out0(new_n181));
  nand02aa1d08x5               g086(.a(new_n127), .b(new_n181), .o1(new_n182));
  nanp02aa1n02x5               g087(.a(new_n174), .b(new_n168), .o1(new_n183));
  oaoi13aa1n04x5               g088(.a(new_n183), .b(new_n166), .c(\a[15] ), .d(\b[14] ), .o1(new_n184));
  aoi112aa1n09x5               g089(.a(new_n173), .b(new_n184), .c(new_n152), .d(new_n180), .o1(new_n185));
  nor002aa1d32x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  nand42aa1d28x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n182), .c(new_n185), .out0(\s[17] ));
  inv000aa1d42x5               g094(.a(\a[18] ), .o1(new_n190));
  nand02aa1d06x5               g095(.a(new_n182), .b(new_n185), .o1(new_n191));
  tech160nm_fiaoi012aa1n05x5   g096(.a(new_n186), .b(new_n191), .c(new_n188), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  nanp02aa1n03x5               g098(.a(new_n152), .b(new_n180), .o1(new_n194));
  nona22aa1n02x4               g099(.a(new_n194), .b(new_n184), .c(new_n173), .out0(new_n195));
  nor002aa1d32x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nand42aa1d28x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nano23aa1d15x5               g102(.a(new_n186), .b(new_n196), .c(new_n197), .d(new_n187), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n195), .c(new_n127), .d(new_n181), .o1(new_n199));
  oa0012aa1n02x5               g104(.a(new_n197), .b(new_n196), .c(new_n186), .o(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  nor042aa1d18x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nand22aa1n12x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norb02aa1d27x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n199), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  tech160nm_finand02aa1n03p5x5 g111(.a(new_n199), .b(new_n201), .o1(new_n207));
  nor042aa1d18x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand02aa1d28x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanb02aa1d36x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n202), .c(new_n207), .d(new_n203), .o1(new_n211));
  aoai13aa1n03x5               g116(.a(new_n204), .b(new_n200), .c(new_n191), .d(new_n198), .o1(new_n212));
  nona22aa1n03x5               g117(.a(new_n212), .b(new_n210), .c(new_n202), .out0(new_n213));
  nanp02aa1n03x5               g118(.a(new_n213), .b(new_n211), .o1(\s[20] ));
  nanb03aa1d24x5               g119(.a(new_n210), .b(new_n198), .c(new_n204), .out0(new_n215));
  nanb03aa1n09x5               g120(.a(new_n208), .b(new_n209), .c(new_n203), .out0(new_n216));
  orn002aa1n03x5               g121(.a(\a[19] ), .b(\b[18] ), .o(new_n217));
  oai112aa1n06x5               g122(.a(new_n217), .b(new_n197), .c(new_n196), .d(new_n186), .o1(new_n218));
  aoi012aa1n12x5               g123(.a(new_n208), .b(new_n202), .c(new_n209), .o1(new_n219));
  oai012aa1n18x5               g124(.a(new_n219), .b(new_n218), .c(new_n216), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n215), .c(new_n182), .d(new_n185), .o1(new_n222));
  nor002aa1n16x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nand42aa1n20x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n215), .o1(new_n226));
  aoi112aa1n02x5               g131(.a(new_n225), .b(new_n220), .c(new_n191), .d(new_n226), .o1(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n222), .c(new_n225), .o1(\s[21] ));
  nor042aa1n09x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  nand42aa1d28x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n223), .c(new_n222), .d(new_n225), .o1(new_n233));
  nand02aa1d04x5               g138(.a(new_n222), .b(new_n225), .o1(new_n234));
  nona22aa1n03x5               g139(.a(new_n234), .b(new_n232), .c(new_n223), .out0(new_n235));
  nanp02aa1n03x5               g140(.a(new_n235), .b(new_n233), .o1(\s[22] ));
  nano23aa1d15x5               g141(.a(new_n223), .b(new_n229), .c(new_n230), .d(new_n224), .out0(new_n237));
  nanb02aa1n02x5               g142(.a(new_n215), .b(new_n237), .out0(new_n238));
  nano22aa1n03x7               g143(.a(new_n208), .b(new_n203), .c(new_n209), .out0(new_n239));
  tech160nm_fioai012aa1n03p5x5 g144(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .o1(new_n240));
  oab012aa1n06x5               g145(.a(new_n240), .b(new_n186), .c(new_n196), .out0(new_n241));
  inv020aa1n03x5               g146(.a(new_n219), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n237), .b(new_n242), .c(new_n241), .d(new_n239), .o1(new_n243));
  aoi012aa1d24x5               g148(.a(new_n229), .b(new_n223), .c(new_n230), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(new_n243), .b(new_n244), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n238), .c(new_n182), .d(new_n185), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  xorc02aa1n12x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  tech160nm_fixnrc02aa1n05x5   g155(.a(\b[23] ), .b(\a[24] ), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n252));
  nand02aa1d04x5               g157(.a(new_n247), .b(new_n250), .o1(new_n253));
  nona22aa1n03x5               g158(.a(new_n253), .b(new_n251), .c(new_n249), .out0(new_n254));
  nanp02aa1n03x5               g159(.a(new_n254), .b(new_n252), .o1(\s[24] ));
  norb02aa1n12x5               g160(.a(new_n250), .b(new_n251), .out0(new_n256));
  nanb03aa1n03x5               g161(.a(new_n215), .b(new_n256), .c(new_n237), .out0(new_n257));
  inv040aa1n02x5               g162(.a(new_n256), .o1(new_n258));
  orn002aa1n02x5               g163(.a(\a[23] ), .b(\b[22] ), .o(new_n259));
  oao003aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .carry(new_n260));
  aoai13aa1n12x5               g165(.a(new_n260), .b(new_n258), .c(new_n243), .d(new_n244), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n257), .c(new_n182), .d(new_n185), .o1(new_n263));
  xorb03aa1n02x5               g168(.a(new_n263), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  xorc02aa1n12x5               g170(.a(\a[25] ), .b(\b[24] ), .out0(new_n266));
  tech160nm_fixnrc02aa1n05x5   g171(.a(\b[25] ), .b(\a[26] ), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n265), .c(new_n263), .d(new_n266), .o1(new_n268));
  nand02aa1d04x5               g173(.a(new_n263), .b(new_n266), .o1(new_n269));
  nona22aa1n03x5               g174(.a(new_n269), .b(new_n267), .c(new_n265), .out0(new_n270));
  nanp02aa1n03x5               g175(.a(new_n270), .b(new_n268), .o1(\s[26] ));
  norb02aa1n03x4               g176(.a(new_n266), .b(new_n267), .out0(new_n272));
  inv000aa1n03x5               g177(.a(new_n272), .o1(new_n273));
  nano23aa1d12x5               g178(.a(new_n273), .b(new_n215), .c(new_n256), .d(new_n237), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n195), .c(new_n127), .d(new_n181), .o1(new_n275));
  inv000aa1d42x5               g180(.a(\a[26] ), .o1(new_n276));
  inv000aa1d42x5               g181(.a(\b[25] ), .o1(new_n277));
  oaoi03aa1n02x5               g182(.a(new_n276), .b(new_n277), .c(new_n265), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  aoi012aa1n09x5               g184(.a(new_n279), .b(new_n261), .c(new_n272), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n275), .out0(\s[27] ));
  tech160nm_finand02aa1n03p5x5 g187(.a(new_n280), .b(new_n275), .o1(new_n283));
  norp02aa1n02x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  norp02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .o1(new_n285));
  nanp02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .o1(new_n286));
  nanb02aa1n12x5               g191(.a(new_n285), .b(new_n286), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n284), .c(new_n283), .d(new_n281), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n244), .o1(new_n289));
  aoai13aa1n06x5               g194(.a(new_n256), .b(new_n289), .c(new_n220), .d(new_n237), .o1(new_n290));
  aoai13aa1n06x5               g195(.a(new_n278), .b(new_n273), .c(new_n290), .d(new_n260), .o1(new_n291));
  aoai13aa1n02x7               g196(.a(new_n281), .b(new_n291), .c(new_n191), .d(new_n274), .o1(new_n292));
  nona22aa1n02x5               g197(.a(new_n292), .b(new_n287), .c(new_n284), .out0(new_n293));
  nanp02aa1n03x5               g198(.a(new_n288), .b(new_n293), .o1(\s[28] ));
  norb02aa1d21x5               g199(.a(new_n281), .b(new_n287), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n191), .d(new_n274), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n295), .o1(new_n297));
  oai012aa1n02x5               g202(.a(new_n286), .b(new_n285), .c(new_n284), .o1(new_n298));
  aoai13aa1n02x7               g203(.a(new_n298), .b(new_n297), .c(new_n280), .d(new_n275), .o1(new_n299));
  norp02aa1n02x5               g204(.a(\b[28] ), .b(\a[29] ), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .o1(new_n301));
  norb02aa1n02x5               g206(.a(new_n301), .b(new_n300), .out0(new_n302));
  oai022aa1n02x5               g207(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n303));
  aboi22aa1n03x5               g208(.a(new_n300), .b(new_n301), .c(new_n303), .d(new_n286), .out0(new_n304));
  aoi022aa1n03x5               g209(.a(new_n299), .b(new_n302), .c(new_n296), .d(new_n304), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d21x5               g211(.a(new_n287), .b(new_n281), .c(new_n302), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n291), .c(new_n191), .d(new_n274), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n307), .o1(new_n309));
  aoi013aa1n02x4               g214(.a(new_n300), .b(new_n303), .c(new_n286), .d(new_n301), .o1(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n309), .c(new_n280), .d(new_n275), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .out0(new_n312));
  aoi113aa1n02x5               g217(.a(new_n312), .b(new_n300), .c(new_n303), .d(new_n301), .e(new_n286), .o1(new_n313));
  aoi022aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n308), .d(new_n313), .o1(\s[30] ));
  nand23aa1n03x5               g219(.a(new_n295), .b(new_n302), .c(new_n312), .o1(new_n315));
  nanb02aa1n03x5               g220(.a(new_n315), .b(new_n283), .out0(new_n316));
  xorc02aa1n02x5               g221(.a(\a[31] ), .b(\b[30] ), .out0(new_n317));
  oao003aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n310), .carry(new_n318));
  norb02aa1n02x5               g223(.a(new_n318), .b(new_n317), .out0(new_n319));
  aoai13aa1n02x7               g224(.a(new_n318), .b(new_n315), .c(new_n280), .d(new_n275), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n316), .b(new_n319), .c(new_n320), .d(new_n317), .o1(\s[31] ));
  xnrb03aa1n02x5               g226(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanp02aa1n03x5               g227(.a(new_n107), .b(new_n109), .o1(new_n323));
  nanb02aa1n02x5               g228(.a(new_n102), .b(new_n106), .out0(new_n324));
  aoib12aa1n02x5               g229(.a(new_n108), .b(new_n104), .c(new_n103), .out0(new_n325));
  aboi22aa1n03x5               g230(.a(new_n103), .b(new_n323), .c(new_n324), .d(new_n325), .out0(\s[4] ));
  xnbna2aa1n03x5               g231(.a(new_n118), .b(new_n107), .c(new_n109), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g232(.a(new_n121), .b(new_n122), .c(new_n323), .o1(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oab012aa1n03x5               g234(.a(new_n116), .b(new_n328), .c(new_n117), .out0(new_n330));
  xnbna2aa1n03x5               g235(.a(new_n330), .b(new_n112), .c(new_n113), .out0(\s[7] ));
  inv000aa1d42x5               g236(.a(new_n113), .o1(new_n332));
  aoi112aa1n03x5               g237(.a(new_n332), .b(new_n110), .c(new_n330), .d(new_n112), .o1(new_n333));
  aoai13aa1n04x5               g238(.a(new_n110), .b(new_n332), .c(new_n330), .d(new_n112), .o1(new_n334));
  nanb02aa1n03x5               g239(.a(new_n333), .b(new_n334), .out0(\s[8] ));
  aoib12aa1n02x5               g240(.a(new_n128), .b(new_n323), .c(new_n119), .out0(new_n336));
  aoi022aa1n02x5               g241(.a(new_n336), .b(new_n126), .c(new_n127), .d(new_n128), .o1(\s[9] ));
endmodule


