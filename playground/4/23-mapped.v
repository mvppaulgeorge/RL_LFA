// Benchmark "adder" written by ABC on Wed Jul 17 14:09:42 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n299,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n320, new_n321, new_n323, new_n324,
    new_n325, new_n326, new_n329, new_n330, new_n332, new_n333, new_n335,
    new_n336, new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n10x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nand42aa1n03x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  nor042aa1n06x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi022aa1n06x5               g007(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n103));
  nor002aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n03x4               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  oai022aa1n02x5               g011(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n107));
  oaoi13aa1n06x5               g012(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n102), .o1(new_n108));
  and002aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o(new_n109));
  nanp02aa1n24x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  oai012aa1n02x5               g015(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .o1(new_n111));
  nor002aa1n02x5               g016(.a(new_n111), .b(new_n109), .o1(new_n112));
  nor002aa1n03x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  aoi012aa1n02x5               g018(.a(new_n113), .b(\a[4] ), .c(\b[3] ), .o1(new_n114));
  nanp02aa1n09x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n115), .b(\b[6] ), .c(\a[7] ), .o1(new_n116));
  nanp02aa1n04x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  oai012aa1n02x5               g022(.a(new_n117), .b(\b[5] ), .c(\a[6] ), .o1(new_n118));
  nor042aa1n02x5               g023(.a(new_n118), .b(new_n116), .o1(new_n119));
  nand23aa1n03x5               g024(.a(new_n119), .b(new_n112), .c(new_n114), .o1(new_n120));
  nor002aa1n03x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nano22aa1n03x7               g026(.a(new_n121), .b(new_n110), .c(new_n117), .out0(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  norb02aa1n03x5               g028(.a(new_n115), .b(new_n113), .out0(new_n124));
  tech160nm_fiao0012aa1n02p5x5 g029(.a(new_n113), .b(new_n121), .c(new_n115), .o(new_n125));
  aoi013aa1n06x4               g030(.a(new_n125), .b(new_n122), .c(new_n123), .d(new_n124), .o1(new_n126));
  oai012aa1n12x5               g031(.a(new_n126), .b(new_n120), .c(new_n108), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n100), .b(new_n127), .c(new_n101), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\a[10] ), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\b[9] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n130), .b(new_n129), .o1(new_n131));
  oai112aa1n02x5               g036(.a(new_n131), .b(new_n98), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  tech160nm_fiao0012aa1n02p5x5 g037(.a(new_n132), .b(new_n127), .c(new_n101), .o(new_n133));
  oai012aa1n02x5               g038(.a(new_n133), .b(new_n128), .c(new_n99), .o1(\s[10] ));
  nand02aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor042aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n136), .b(\a[10] ), .c(\b[9] ), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n136), .o1(new_n138));
  aoi022aa1n02x5               g043(.a(new_n133), .b(new_n98), .c(new_n138), .d(new_n135), .o1(new_n139));
  aoi013aa1n02x4               g044(.a(new_n139), .b(new_n137), .c(new_n135), .d(new_n133), .o1(\s[11] ));
  nanp03aa1n02x5               g045(.a(new_n133), .b(new_n135), .c(new_n137), .o1(new_n141));
  nor042aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n141), .c(new_n138), .out0(\s[12] ));
  nano23aa1n02x5               g050(.a(new_n142), .b(new_n136), .c(new_n143), .d(new_n135), .out0(new_n146));
  nano23aa1n02x5               g051(.a(new_n97), .b(new_n100), .c(new_n101), .d(new_n98), .out0(new_n147));
  and002aa1n02x5               g052(.a(new_n147), .b(new_n146), .o(new_n148));
  nanb03aa1n02x5               g053(.a(new_n142), .b(new_n143), .c(new_n135), .out0(new_n149));
  oai112aa1n03x5               g054(.a(new_n138), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n150));
  aoi012aa1n02x7               g055(.a(new_n142), .b(new_n136), .c(new_n143), .o1(new_n151));
  oai012aa1n06x5               g056(.a(new_n151), .b(new_n150), .c(new_n149), .o1(new_n152));
  nor042aa1n12x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1n03x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n152), .c(new_n127), .d(new_n148), .o1(new_n157));
  aoi112aa1n02x5               g062(.a(new_n156), .b(new_n152), .c(new_n127), .d(new_n148), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(\s[13] ));
  inv000aa1d42x5               g064(.a(new_n153), .o1(new_n160));
  nor042aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n06x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  xobna2aa1n03x5               g068(.a(new_n163), .b(new_n157), .c(new_n160), .out0(\s[14] ));
  nano23aa1n09x5               g069(.a(new_n153), .b(new_n161), .c(new_n162), .d(new_n154), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n152), .c(new_n127), .d(new_n148), .o1(new_n166));
  tech160nm_fiaoi012aa1n04x5   g071(.a(new_n161), .b(new_n153), .c(new_n162), .o1(new_n167));
  nor042aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n166), .c(new_n167), .out0(\s[15] ));
  aob012aa1n02x5               g077(.a(new_n171), .b(new_n166), .c(new_n167), .out0(new_n173));
  nor042aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand22aa1n03x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  aoib12aa1n02x5               g081(.a(new_n168), .b(new_n175), .c(new_n174), .out0(new_n177));
  oai012aa1n02x5               g082(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .o1(new_n178));
  aboi22aa1n03x5               g083(.a(new_n176), .b(new_n178), .c(new_n173), .d(new_n177), .out0(\s[16] ));
  nano23aa1n06x5               g084(.a(new_n168), .b(new_n174), .c(new_n175), .d(new_n169), .out0(new_n180));
  nand22aa1n03x5               g085(.a(new_n180), .b(new_n165), .o1(new_n181));
  nano22aa1n03x7               g086(.a(new_n181), .b(new_n146), .c(new_n147), .out0(new_n182));
  nand02aa1d04x5               g087(.a(new_n127), .b(new_n182), .o1(new_n183));
  inv020aa1n02x5               g088(.a(new_n167), .o1(new_n184));
  tech160nm_fiao0012aa1n03p5x5 g089(.a(new_n174), .b(new_n168), .c(new_n175), .o(new_n185));
  tech160nm_fiaoi012aa1n05x5   g090(.a(new_n185), .b(new_n180), .c(new_n184), .o1(new_n186));
  inv040aa1n02x5               g091(.a(new_n186), .o1(new_n187));
  aoib12aa1n06x5               g092(.a(new_n187), .b(new_n152), .c(new_n181), .out0(new_n188));
  nanp02aa1n06x5               g093(.a(new_n183), .b(new_n188), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  norp02aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\a[17] ), .o1(new_n193));
  oaib12aa1n02x5               g098(.a(new_n189), .b(new_n193), .c(\b[16] ), .out0(new_n194));
  xorc02aa1n02x5               g099(.a(\a[18] ), .b(\b[17] ), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n194), .c(new_n192), .out0(\s[18] ));
  nano22aa1n02x4               g101(.a(new_n142), .b(new_n135), .c(new_n143), .out0(new_n197));
  nanp03aa1n02x5               g102(.a(new_n197), .b(new_n132), .c(new_n137), .o1(new_n198));
  aoai13aa1n06x5               g103(.a(new_n186), .b(new_n181), .c(new_n198), .d(new_n151), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\a[18] ), .o1(new_n200));
  xroi22aa1d06x4               g105(.a(new_n193), .b(\b[16] ), .c(new_n200), .d(\b[17] ), .out0(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n199), .c(new_n127), .d(new_n182), .o1(new_n202));
  nor042aa1n02x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  aoi112aa1n09x5               g108(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n204));
  norp02aa1n02x5               g109(.a(new_n204), .b(new_n203), .o1(new_n205));
  nor042aa1n06x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nand02aa1n08x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  norb02aa1n12x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n202), .c(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n02x5               g115(.a(new_n208), .b(new_n202), .c(new_n205), .out0(new_n211));
  nor042aa1n06x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand02aa1d24x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n12x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoib12aa1n02x5               g119(.a(new_n206), .b(new_n213), .c(new_n212), .out0(new_n215));
  oai012aa1n02x5               g120(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .o1(new_n216));
  aoi022aa1n02x5               g121(.a(new_n216), .b(new_n214), .c(new_n211), .d(new_n215), .o1(\s[20] ));
  nano23aa1n02x5               g122(.a(new_n206), .b(new_n212), .c(new_n213), .d(new_n207), .out0(new_n218));
  nand02aa1d04x5               g123(.a(new_n201), .b(new_n218), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n199), .c(new_n127), .d(new_n182), .o1(new_n221));
  oai112aa1n06x5               g126(.a(new_n208), .b(new_n214), .c(new_n204), .d(new_n203), .o1(new_n222));
  tech160nm_fiaoi012aa1n04x5   g127(.a(new_n212), .b(new_n206), .c(new_n213), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(new_n222), .b(new_n223), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  tech160nm_fixnrc02aa1n05x5   g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  xobna2aa1n03x5               g131(.a(new_n226), .b(new_n221), .c(new_n225), .out0(\s[21] ));
  tech160nm_fiao0012aa1n03p5x5 g132(.a(new_n226), .b(new_n221), .c(new_n225), .o(new_n228));
  xorc02aa1n12x5               g133(.a(\a[22] ), .b(\b[21] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(\a[21] ), .o1(new_n230));
  aoib12aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(\b[20] ), .out0(new_n231));
  oaib12aa1n02x5               g136(.a(new_n228), .b(\b[20] ), .c(new_n230), .out0(new_n232));
  aoi022aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n228), .d(new_n231), .o1(\s[22] ));
  nanb02aa1d24x5               g138(.a(new_n226), .b(new_n229), .out0(new_n234));
  inv000aa1d42x5               g139(.a(\a[22] ), .o1(new_n235));
  aoi112aa1n03x4               g140(.a(\b[20] ), .b(\a[21] ), .c(\a[22] ), .d(\b[21] ), .o1(new_n236));
  aoib12aa1n06x5               g141(.a(new_n236), .b(new_n235), .c(\b[21] ), .out0(new_n237));
  aoai13aa1n12x5               g142(.a(new_n237), .b(new_n234), .c(new_n222), .d(new_n223), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  nona22aa1n02x4               g144(.a(new_n189), .b(new_n219), .c(new_n234), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xnbna2aa1n03x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .out0(\s[23] ));
  aob012aa1n03x5               g147(.a(new_n241), .b(new_n240), .c(new_n239), .out0(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  norp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  norp02aa1n02x5               g150(.a(new_n244), .b(new_n245), .o1(new_n246));
  xroi22aa1d04x5               g151(.a(new_n230), .b(\b[20] ), .c(new_n235), .d(\b[21] ), .out0(new_n247));
  aoi013aa1n02x4               g152(.a(new_n238), .b(new_n189), .c(new_n220), .d(new_n247), .o1(new_n248));
  oaoi03aa1n02x5               g153(.a(\a[23] ), .b(\b[22] ), .c(new_n248), .o1(new_n249));
  aoi022aa1n02x5               g154(.a(new_n249), .b(new_n244), .c(new_n243), .d(new_n246), .o1(\s[24] ));
  inv000aa1d42x5               g155(.a(\a[23] ), .o1(new_n251));
  inv040aa1d32x5               g156(.a(\a[24] ), .o1(new_n252));
  xroi22aa1d06x4               g157(.a(new_n251), .b(\b[22] ), .c(new_n252), .d(\b[23] ), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  nona32aa1n09x5               g159(.a(new_n189), .b(new_n254), .c(new_n234), .d(new_n219), .out0(new_n255));
  inv000aa1d42x5               g160(.a(\b[23] ), .o1(new_n256));
  tech160nm_fioaoi03aa1n02p5x5 g161(.a(new_n252), .b(new_n256), .c(new_n245), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  aoi012aa1n03x5               g163(.a(new_n258), .b(new_n238), .c(new_n253), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  aob012aa1n03x5               g165(.a(new_n260), .b(new_n255), .c(new_n259), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n260), .b(new_n258), .c(new_n238), .d(new_n253), .o1(new_n262));
  aobi12aa1n02x7               g167(.a(new_n261), .b(new_n262), .c(new_n255), .out0(\s[25] ));
  xorc02aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  nor042aa1n06x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  norp02aa1n02x5               g170(.a(new_n264), .b(new_n265), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n265), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n260), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n267), .b(new_n268), .c(new_n255), .d(new_n259), .o1(new_n269));
  aoi022aa1n02x5               g174(.a(new_n269), .b(new_n264), .c(new_n261), .d(new_n266), .o1(\s[26] ));
  nanp02aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[25] ), .b(\a[26] ), .out0(new_n272));
  nano22aa1n02x5               g177(.a(new_n272), .b(new_n267), .c(new_n271), .out0(new_n273));
  aoai13aa1n09x5               g178(.a(new_n273), .b(new_n258), .c(new_n238), .d(new_n253), .o1(new_n274));
  nano32aa1n06x5               g179(.a(new_n219), .b(new_n273), .c(new_n247), .d(new_n253), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n199), .c(new_n127), .d(new_n182), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n267), .carry(new_n277));
  nand23aa1n06x5               g182(.a(new_n274), .b(new_n276), .c(new_n277), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  and002aa1n02x5               g184(.a(new_n277), .b(new_n279), .o(new_n280));
  nanp03aa1n02x5               g185(.a(new_n274), .b(new_n276), .c(new_n280), .o1(new_n281));
  oaib12aa1n02x5               g186(.a(new_n281), .b(new_n279), .c(new_n278), .out0(\s[27] ));
  inv000aa1d42x5               g187(.a(\a[27] ), .o1(new_n283));
  oaib12aa1n06x5               g188(.a(new_n278), .b(new_n283), .c(\b[26] ), .out0(new_n284));
  oaib12aa1n06x5               g189(.a(new_n284), .b(\b[26] ), .c(new_n283), .out0(new_n285));
  xorc02aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .out0(new_n286));
  nor042aa1n03x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  aoi022aa1n02x7               g193(.a(new_n285), .b(new_n286), .c(new_n284), .d(new_n288), .o1(\s[28] ));
  inv000aa1d42x5               g194(.a(\a[28] ), .o1(new_n290));
  xroi22aa1d04x5               g195(.a(new_n283), .b(\b[26] ), .c(new_n290), .d(\b[27] ), .out0(new_n291));
  nand02aa1d04x5               g196(.a(new_n278), .b(new_n291), .o1(new_n292));
  inv000aa1d42x5               g197(.a(\b[27] ), .o1(new_n293));
  oaoi03aa1n09x5               g198(.a(new_n290), .b(new_n293), .c(new_n287), .o1(new_n294));
  nanp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .out0(new_n296));
  norb02aa1n02x5               g201(.a(new_n294), .b(new_n296), .out0(new_n297));
  aoi022aa1n03x5               g202(.a(new_n295), .b(new_n296), .c(new_n292), .d(new_n297), .o1(\s[29] ));
  nanp02aa1n02x5               g203(.a(\b[0] ), .b(\a[1] ), .o1(new_n299));
  xorb03aa1n02x5               g204(.a(new_n299), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g205(.a(new_n279), .b(new_n296), .c(new_n286), .o(new_n301));
  nand02aa1d04x5               g206(.a(new_n278), .b(new_n301), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  nanp02aa1n03x5               g208(.a(new_n302), .b(new_n303), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .out0(new_n305));
  aoi012aa1n02x5               g210(.a(new_n294), .b(\a[29] ), .c(\b[28] ), .o1(new_n306));
  oabi12aa1n02x5               g211(.a(new_n305), .b(\a[29] ), .c(\b[28] ), .out0(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n306), .o1(new_n308));
  aoi022aa1n03x5               g213(.a(new_n304), .b(new_n305), .c(new_n302), .d(new_n308), .o1(\s[30] ));
  nanb02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nanb02aa1n02x5               g215(.a(\a[31] ), .b(\b[30] ), .out0(new_n311));
  and003aa1n02x5               g216(.a(new_n291), .b(new_n305), .c(new_n296), .o(new_n312));
  nand42aa1n02x5               g217(.a(new_n278), .b(new_n312), .o1(new_n313));
  tech160nm_fioaoi03aa1n03p5x5 g218(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n314), .o1(new_n315));
  aoi022aa1n02x7               g220(.a(new_n313), .b(new_n315), .c(new_n311), .d(new_n310), .o1(new_n316));
  nanp02aa1n02x5               g221(.a(new_n311), .b(new_n310), .o1(new_n317));
  aoi112aa1n03x4               g222(.a(new_n317), .b(new_n314), .c(new_n278), .d(new_n312), .o1(new_n318));
  nor002aa1n02x5               g223(.a(new_n316), .b(new_n318), .o1(\s[31] ));
  inv000aa1d42x5               g224(.a(new_n102), .o1(new_n320));
  nanb02aa1n02x5               g225(.a(new_n103), .b(new_n320), .out0(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  norp02aa1n02x5               g227(.a(\b[3] ), .b(\a[4] ), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[4] ), .b(\b[3] ), .out0(new_n324));
  aoi112aa1n02x5               g229(.a(new_n104), .b(new_n324), .c(new_n321), .d(new_n106), .o1(new_n325));
  tech160nm_fiao0012aa1n02p5x5 g230(.a(new_n108), .b(\a[4] ), .c(\b[3] ), .o(new_n326));
  oab012aa1n02x4               g231(.a(new_n325), .b(new_n326), .c(new_n323), .out0(\s[4] ));
  xnrb03aa1n02x5               g232(.a(new_n326), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  xorc02aa1n02x5               g233(.a(\a[6] ), .b(\b[5] ), .out0(new_n329));
  oao003aa1n02x5               g234(.a(\a[5] ), .b(\b[4] ), .c(new_n326), .carry(new_n330));
  xnrc02aa1n02x5               g235(.a(new_n330), .b(new_n329), .out0(\s[6] ));
  norb02aa1n02x5               g236(.a(new_n117), .b(new_n121), .out0(new_n332));
  nanp02aa1n02x5               g237(.a(new_n330), .b(new_n329), .o1(new_n333));
  xobna2aa1n03x5               g238(.a(new_n332), .b(new_n333), .c(new_n110), .out0(\s[7] ));
  nanp02aa1n02x5               g239(.a(new_n333), .b(new_n122), .o1(new_n335));
  oai012aa1n02x5               g240(.a(new_n335), .b(\b[6] ), .c(\a[7] ), .o1(new_n336));
  aoib12aa1n02x5               g241(.a(new_n121), .b(new_n115), .c(new_n113), .out0(new_n337));
  aoi022aa1n02x5               g242(.a(new_n336), .b(new_n124), .c(new_n335), .d(new_n337), .o1(\s[8] ));
  xorb03aa1n02x5               g243(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


