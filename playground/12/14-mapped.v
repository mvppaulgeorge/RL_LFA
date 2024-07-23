// Benchmark "adder" written by ABC on Wed Jul 17 18:11:58 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n307, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n329, new_n330, new_n332,
    new_n333, new_n334, new_n336, new_n338, new_n340;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1n20x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  norp02aa1n12x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand02aa1d28x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor022aa1n16x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand42aa1n16x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n06x5               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  nona23aa1n09x5               g009(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n105));
  oai022aa1d24x5               g010(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n106));
  aob012aa1d15x5               g011(.a(new_n106), .b(\b[5] ), .c(\a[6] ), .out0(new_n107));
  oa0012aa1n06x5               g012(.a(new_n101), .b(new_n102), .c(new_n100), .o(new_n108));
  oabi12aa1n06x5               g013(.a(new_n108), .b(new_n105), .c(new_n107), .out0(new_n109));
  oai022aa1n09x5               g014(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n110));
  nor042aa1n09x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand02aa1d16x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  norb02aa1n06x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor002aa1n04x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  aoi022aa1d24x5               g019(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n115));
  tech160nm_fioai012aa1n04x5   g020(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n116));
  tech160nm_fixorc02aa1n05x5   g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  and002aa1n06x5               g022(.a(\b[4] ), .b(\a[5] ), .o(new_n118));
  nand42aa1d28x5               g023(.a(\b[3] ), .b(\a[4] ), .o1(new_n119));
  oai012aa1d24x5               g024(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .o1(new_n120));
  nor042aa1n06x5               g025(.a(new_n120), .b(new_n118), .o1(new_n121));
  nand02aa1n06x5               g026(.a(new_n121), .b(new_n117), .o1(new_n122));
  aoib12aa1n06x5               g027(.a(new_n122), .b(new_n116), .c(new_n110), .out0(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n109), .c(new_n123), .d(new_n104), .o1(new_n126));
  nor002aa1d32x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1d28x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n15x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n99), .out0(\s[10] ));
  aob012aa1n03x5               g035(.a(new_n129), .b(new_n126), .c(new_n99), .out0(new_n131));
  aoai13aa1n12x5               g036(.a(new_n128), .b(new_n127), .c(new_n97), .d(new_n98), .o1(new_n132));
  xorc02aa1n12x5               g037(.a(\a[11] ), .b(\b[10] ), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n131), .c(new_n132), .out0(\s[11] ));
  inv000aa1d42x5               g039(.a(new_n129), .o1(new_n135));
  aoai13aa1n06x5               g040(.a(new_n132), .b(new_n135), .c(new_n126), .d(new_n99), .o1(new_n136));
  inv040aa1d30x5               g041(.a(\a[11] ), .o1(new_n137));
  inv040aa1d30x5               g042(.a(\b[10] ), .o1(new_n138));
  nand22aa1n09x5               g043(.a(new_n138), .b(new_n137), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  orn002aa1n24x5               g045(.a(\a[12] ), .b(\b[11] ), .o(new_n141));
  nand02aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  tech160nm_finand02aa1n05x5   g047(.a(new_n141), .b(new_n142), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n140), .c(new_n136), .d(new_n133), .o1(new_n144));
  aoi112aa1n03x5               g049(.a(new_n140), .b(new_n143), .c(new_n136), .d(new_n133), .o1(new_n145));
  nanb02aa1n03x5               g050(.a(new_n145), .b(new_n144), .out0(\s[12] ));
  nona23aa1d18x5               g051(.a(new_n129), .b(new_n133), .c(new_n124), .d(new_n143), .out0(new_n147));
  inv040aa1n03x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n109), .c(new_n123), .d(new_n104), .o1(new_n149));
  and002aa1n02x5               g054(.a(\b[10] ), .b(\a[11] ), .o(new_n150));
  aoai13aa1n12x5               g055(.a(new_n141), .b(new_n150), .c(new_n132), .d(new_n139), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n142), .o1(new_n152));
  nanp02aa1n03x5               g057(.a(new_n149), .b(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g059(.a(\a[13] ), .o1(new_n155));
  inv000aa1d42x5               g060(.a(\b[12] ), .o1(new_n156));
  oaoi03aa1n03x5               g061(.a(new_n155), .b(new_n156), .c(new_n153), .o1(new_n157));
  xnrb03aa1n03x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n24x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp02aa1n06x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nor002aa1d32x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand02aa1d06x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1d18x5               g067(.a(new_n162), .b(new_n160), .c(new_n159), .d(new_n161), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n162), .b(new_n161), .c(new_n155), .d(new_n156), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n163), .c(new_n149), .d(new_n152), .o1(new_n165));
  nor042aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand22aa1n09x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  oai022aa1n02x5               g073(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n169));
  aboi22aa1n03x5               g074(.a(new_n166), .b(new_n167), .c(new_n169), .d(new_n162), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n163), .c(new_n149), .d(new_n152), .o1(new_n171));
  aobi12aa1n02x5               g076(.a(new_n171), .b(new_n168), .c(new_n165), .out0(\s[15] ));
  nor042aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand22aa1n03x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n166), .c(new_n165), .d(new_n167), .o1(new_n176));
  aoi112aa1n03x4               g081(.a(new_n166), .b(new_n175), .c(new_n165), .d(new_n167), .o1(new_n177));
  nanb02aa1n03x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  aoib12aa1n06x5               g083(.a(new_n108), .b(new_n104), .c(new_n107), .out0(new_n179));
  oaoi13aa1n12x5               g084(.a(new_n110), .b(new_n113), .c(new_n115), .d(new_n114), .o1(new_n180));
  oai013aa1d12x5               g085(.a(new_n179), .b(new_n180), .c(new_n122), .d(new_n105), .o1(new_n181));
  nona23aa1n09x5               g086(.a(new_n174), .b(new_n167), .c(new_n166), .d(new_n173), .out0(new_n182));
  nor042aa1n06x5               g087(.a(new_n182), .b(new_n163), .o1(new_n183));
  norb02aa1n15x5               g088(.a(new_n183), .b(new_n147), .out0(new_n184));
  nand02aa1d08x5               g089(.a(new_n181), .b(new_n184), .o1(new_n185));
  aoai13aa1n03x5               g090(.a(new_n167), .b(new_n166), .c(new_n169), .d(new_n162), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(\a[16] ), .b(\b[15] ), .c(new_n186), .o1(new_n187));
  aoi013aa1n09x5               g092(.a(new_n187), .b(new_n151), .c(new_n183), .d(new_n142), .o1(new_n188));
  xorc02aa1n12x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n185), .c(new_n188), .out0(\s[17] ));
  inv040aa1d32x5               g095(.a(\a[17] ), .o1(new_n191));
  inv040aa1n12x5               g096(.a(\b[16] ), .o1(new_n192));
  nand02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nanp03aa1n03x5               g098(.a(new_n151), .b(new_n183), .c(new_n142), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(\a[15] ), .b(\b[14] ), .c(new_n164), .o1(new_n195));
  oai012aa1n03x5               g100(.a(new_n174), .b(new_n195), .c(new_n173), .o1(new_n196));
  nand02aa1n03x5               g101(.a(new_n194), .b(new_n196), .o1(new_n197));
  aoai13aa1n03x5               g102(.a(new_n189), .b(new_n197), .c(new_n181), .d(new_n184), .o1(new_n198));
  nor042aa1n04x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  tech160nm_finand02aa1n05x5   g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nanb02aa1n09x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  xobna2aa1n03x5               g106(.a(new_n201), .b(new_n198), .c(new_n193), .out0(\s[18] ));
  norb02aa1n02x5               g107(.a(new_n189), .b(new_n201), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n197), .c(new_n181), .d(new_n184), .o1(new_n204));
  aoai13aa1n12x5               g109(.a(new_n200), .b(new_n199), .c(new_n191), .d(new_n192), .o1(new_n205));
  nor002aa1d32x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nand02aa1n06x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n204), .c(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g115(.a(new_n204), .b(new_n205), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand02aa1d28x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n206), .c(new_n211), .d(new_n208), .o1(new_n215));
  nanp02aa1n06x5               g120(.a(new_n185), .b(new_n188), .o1(new_n216));
  oaoi03aa1n03x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n193), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n208), .b(new_n217), .c(new_n216), .d(new_n203), .o1(new_n218));
  nona22aa1n03x5               g123(.a(new_n218), .b(new_n214), .c(new_n206), .out0(new_n219));
  nanp02aa1n03x5               g124(.a(new_n215), .b(new_n219), .o1(\s[20] ));
  nona23aa1d24x5               g125(.a(new_n213), .b(new_n207), .c(new_n206), .d(new_n212), .out0(new_n221));
  ao0012aa1n12x5               g126(.a(new_n212), .b(new_n206), .c(new_n213), .o(new_n222));
  oabi12aa1n18x5               g127(.a(new_n222), .b(new_n221), .c(new_n205), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  norb03aa1n09x5               g129(.a(new_n189), .b(new_n221), .c(new_n201), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n224), .b(new_n226), .c(new_n185), .d(new_n188), .o1(new_n227));
  xnrc02aa1n12x5               g132(.a(\b[20] ), .b(\a[21] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n216), .c(new_n225), .o1(new_n230));
  aoi022aa1n02x5               g135(.a(new_n230), .b(new_n224), .c(new_n227), .d(new_n229), .o1(\s[21] ));
  nor042aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n232), .c(new_n227), .d(new_n229), .o1(new_n234));
  nand02aa1n02x5               g139(.a(new_n227), .b(new_n229), .o1(new_n235));
  nona22aa1n02x4               g140(.a(new_n235), .b(new_n233), .c(new_n232), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n236), .b(new_n234), .o1(\s[22] ));
  nor042aa1n06x5               g142(.a(new_n233), .b(new_n228), .o1(new_n238));
  nona23aa1n09x5               g143(.a(new_n238), .b(new_n189), .c(new_n221), .d(new_n201), .out0(new_n239));
  nano23aa1n03x7               g144(.a(new_n206), .b(new_n212), .c(new_n213), .d(new_n207), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n238), .b(new_n222), .c(new_n240), .d(new_n217), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\a[22] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[21] ), .o1(new_n243));
  oaoi03aa1n09x5               g148(.a(new_n242), .b(new_n243), .c(new_n232), .o1(new_n244));
  nanp02aa1n04x5               g149(.a(new_n241), .b(new_n244), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n239), .c(new_n185), .d(new_n188), .o1(new_n247));
  xorc02aa1n12x5               g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  nanp02aa1n03x5               g153(.a(new_n247), .b(new_n248), .o1(new_n249));
  aoi113aa1n03x5               g154(.a(new_n245), .b(new_n248), .c(new_n216), .d(new_n225), .e(new_n238), .o1(new_n250));
  norb02aa1n03x4               g155(.a(new_n249), .b(new_n250), .out0(\s[23] ));
  norp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  tech160nm_fixnrc02aa1n05x5   g157(.a(\b[23] ), .b(\a[24] ), .out0(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n252), .c(new_n247), .d(new_n248), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n249), .b(new_n253), .c(new_n252), .out0(new_n255));
  nanp02aa1n03x5               g160(.a(new_n255), .b(new_n254), .o1(\s[24] ));
  norb02aa1n03x5               g161(.a(new_n248), .b(new_n253), .out0(new_n257));
  nand03aa1n02x5               g162(.a(new_n225), .b(new_n238), .c(new_n257), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n257), .o1(new_n259));
  orn002aa1n02x5               g164(.a(\a[23] ), .b(\b[22] ), .o(new_n260));
  oao003aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n260), .carry(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n259), .c(new_n241), .d(new_n244), .o1(new_n262));
  inv040aa1n03x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n258), .c(new_n185), .d(new_n188), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[25] ), .b(\b[24] ), .out0(new_n267));
  xnrc02aa1n12x5               g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n266), .c(new_n264), .d(new_n267), .o1(new_n269));
  nanp02aa1n03x5               g174(.a(new_n264), .b(new_n267), .o1(new_n270));
  nona22aa1n02x4               g175(.a(new_n270), .b(new_n268), .c(new_n266), .out0(new_n271));
  nanp02aa1n03x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  norb02aa1n12x5               g177(.a(new_n267), .b(new_n268), .out0(new_n273));
  nano22aa1n03x7               g178(.a(new_n239), .b(new_n257), .c(new_n273), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n197), .c(new_n181), .d(new_n184), .o1(new_n275));
  nand42aa1n04x5               g180(.a(new_n262), .b(new_n273), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n277));
  oab012aa1n02x4               g182(.a(new_n277), .b(\a[26] ), .c(\b[25] ), .out0(new_n278));
  nanp03aa1n03x5               g183(.a(new_n275), .b(new_n276), .c(new_n278), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n276), .c(new_n278), .out0(new_n281));
  aoi022aa1n02x5               g186(.a(new_n281), .b(new_n275), .c(new_n279), .d(new_n280), .o1(\s[27] ));
  nor042aa1n03x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  xnrc02aa1n12x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n283), .c(new_n279), .d(new_n280), .o1(new_n285));
  aobi12aa1n09x5               g190(.a(new_n274), .b(new_n185), .c(new_n188), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n244), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n257), .b(new_n287), .c(new_n223), .d(new_n238), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n273), .o1(new_n289));
  aoai13aa1n06x5               g194(.a(new_n278), .b(new_n289), .c(new_n288), .d(new_n261), .o1(new_n290));
  oai012aa1n03x5               g195(.a(new_n280), .b(new_n290), .c(new_n286), .o1(new_n291));
  nona22aa1n03x5               g196(.a(new_n291), .b(new_n284), .c(new_n283), .out0(new_n292));
  nanp02aa1n03x5               g197(.a(new_n285), .b(new_n292), .o1(\s[28] ));
  norb02aa1n03x5               g198(.a(new_n280), .b(new_n284), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n286), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[29] ), .b(\b[28] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(\a[28] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\b[27] ), .o1(new_n298));
  aoi112aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(new_n296), .b(new_n299), .c(new_n297), .d(new_n298), .o1(new_n300));
  aobi12aa1n06x5               g205(.a(new_n278), .b(new_n262), .c(new_n273), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n294), .o1(new_n302));
  oao003aa1n06x5               g207(.a(new_n297), .b(new_n298), .c(new_n283), .carry(new_n303));
  inv000aa1d42x5               g208(.a(new_n303), .o1(new_n304));
  aoai13aa1n02x7               g209(.a(new_n304), .b(new_n302), .c(new_n301), .d(new_n275), .o1(new_n305));
  aoi022aa1n03x5               g210(.a(new_n305), .b(new_n296), .c(new_n295), .d(new_n300), .o1(\s[29] ));
  nanp02aa1n02x5               g211(.a(\b[0] ), .b(\a[1] ), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g213(.a(new_n284), .b(new_n280), .c(new_n296), .out0(new_n309));
  oaih12aa1n02x5               g214(.a(new_n309), .b(new_n290), .c(new_n286), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .out0(new_n311));
  inv000aa1d42x5               g216(.a(\a[29] ), .o1(new_n312));
  inv000aa1d42x5               g217(.a(\b[28] ), .o1(new_n313));
  oabi12aa1n02x5               g218(.a(new_n311), .b(\a[29] ), .c(\b[28] ), .out0(new_n314));
  oaoi13aa1n02x5               g219(.a(new_n314), .b(new_n303), .c(new_n312), .d(new_n313), .o1(new_n315));
  inv000aa1n02x5               g220(.a(new_n309), .o1(new_n316));
  tech160nm_fioaoi03aa1n03p5x5 g221(.a(new_n312), .b(new_n313), .c(new_n303), .o1(new_n317));
  aoai13aa1n02x7               g222(.a(new_n317), .b(new_n316), .c(new_n301), .d(new_n275), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n318), .b(new_n311), .c(new_n310), .d(new_n315), .o1(\s[30] ));
  nanp03aa1n02x5               g224(.a(new_n294), .b(new_n296), .c(new_n311), .o1(new_n320));
  oabi12aa1n02x7               g225(.a(new_n320), .b(new_n290), .c(new_n286), .out0(new_n321));
  xorc02aa1n02x5               g226(.a(\a[31] ), .b(\b[30] ), .out0(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n323));
  norb02aa1n02x5               g228(.a(new_n323), .b(new_n322), .out0(new_n324));
  aoai13aa1n02x7               g229(.a(new_n323), .b(new_n320), .c(new_n301), .d(new_n275), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n325), .b(new_n322), .c(new_n321), .d(new_n324), .o1(\s[31] ));
  norp03aa1n02x5               g231(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n116), .b(new_n327), .out0(\s[3] ));
  xorc02aa1n02x5               g233(.a(\a[4] ), .b(\b[3] ), .out0(new_n329));
  norp02aa1n02x5               g234(.a(new_n329), .b(new_n111), .o1(new_n330));
  aboi22aa1n03x5               g235(.a(new_n180), .b(new_n329), .c(new_n330), .d(new_n116), .out0(\s[4] ));
  inv000aa1d42x5               g236(.a(new_n119), .o1(new_n332));
  norb02aa1n02x5               g237(.a(new_n121), .b(new_n180), .out0(new_n333));
  xnrc02aa1n02x5               g238(.a(\b[4] ), .b(\a[5] ), .out0(new_n334));
  oaoi13aa1n02x5               g239(.a(new_n333), .b(new_n334), .c(new_n180), .d(new_n332), .o1(\s[5] ));
  obai22aa1n02x7               g240(.a(new_n121), .b(new_n180), .c(\a[5] ), .d(\b[4] ), .out0(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai012aa1n02x5               g242(.a(new_n107), .b(new_n180), .c(new_n122), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g244(.a(new_n102), .b(new_n338), .c(new_n103), .o1(new_n340));
  xnrb03aa1n02x5               g245(.a(new_n340), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g246(.a(new_n181), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


