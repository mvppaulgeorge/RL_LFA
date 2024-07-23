// Benchmark "adder" written by ABC on Wed Jul 17 22:40:12 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n128, new_n129, new_n130, new_n131, new_n132, new_n133,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n180, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n211, new_n212, new_n213, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n245, new_n246,
    new_n247, new_n248, new_n249, new_n250, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n298, new_n300, new_n302, new_n303, new_n304,
    new_n306, new_n308, new_n310;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d24x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  nanb02aa1n02x5               g005(.a(\b[8] ), .b(new_n100), .out0(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[8] ), .b(\a[9] ), .out0(new_n102));
  oaih22aa1n04x5               g007(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n103));
  nor042aa1d18x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nand22aa1n09x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  norb02aa1n06x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  aoi022aa1n06x5               g011(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n107));
  nanp03aa1n02x5               g012(.a(new_n106), .b(new_n103), .c(new_n107), .o1(new_n108));
  inv040aa1n02x5               g013(.a(new_n104), .o1(new_n109));
  oao003aa1n02x5               g014(.a(\a[8] ), .b(\b[7] ), .c(new_n109), .carry(new_n110));
  nanp02aa1n03x5               g015(.a(new_n110), .b(new_n108), .o1(new_n111));
  and002aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o(new_n112));
  nand22aa1n09x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nor042aa1n06x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nand42aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  oai112aa1n06x5               g021(.a(new_n116), .b(new_n114), .c(new_n115), .d(new_n113), .o1(new_n117));
  oa0022aa1n06x5               g022(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n118));
  nanp02aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nand23aa1n03x5               g024(.a(new_n106), .b(new_n107), .c(new_n119), .o1(new_n120));
  aoi112aa1n06x5               g025(.a(new_n120), .b(new_n112), .c(new_n117), .d(new_n118), .o1(new_n121));
  oabi12aa1n02x5               g026(.a(new_n102), .b(new_n121), .c(new_n111), .out0(new_n122));
  xnbna2aa1n03x5               g027(.a(new_n99), .b(new_n122), .c(new_n101), .out0(\s[10] ));
  inv000aa1d42x5               g028(.a(new_n97), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n98), .o1(new_n125));
  aoi013aa1n03x5               g030(.a(new_n125), .b(new_n122), .c(new_n101), .d(new_n124), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g032(.a(\b[11] ), .b(\a[12] ), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  nand42aa1d28x5               g034(.a(\b[11] ), .b(\a[12] ), .o1(new_n130));
  inv040aa1d32x5               g035(.a(\a[11] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\b[10] ), .o1(new_n132));
  oaoi03aa1n02x5               g037(.a(new_n131), .b(new_n132), .c(new_n126), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n129), .c(new_n130), .out0(\s[12] ));
  nor042aa1n03x5               g039(.a(new_n121), .b(new_n111), .o1(new_n135));
  nand42aa1n08x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nano22aa1n03x7               g041(.a(new_n128), .b(new_n136), .c(new_n130), .out0(new_n137));
  nor042aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb03aa1n03x5               g043(.a(new_n98), .b(new_n97), .c(new_n138), .out0(new_n139));
  nanb03aa1n06x5               g044(.a(new_n102), .b(new_n139), .c(new_n137), .out0(new_n140));
  nanb03aa1n12x5               g045(.a(new_n128), .b(new_n130), .c(new_n136), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n132), .b(new_n131), .o1(new_n142));
  oaih22aa1d12x5               g047(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n143));
  nand23aa1n03x5               g048(.a(new_n143), .b(new_n142), .c(new_n98), .o1(new_n144));
  tech160nm_fiaoi012aa1n03p5x5 g049(.a(new_n128), .b(new_n138), .c(new_n130), .o1(new_n145));
  oai012aa1n06x5               g050(.a(new_n145), .b(new_n144), .c(new_n141), .o1(new_n146));
  oabi12aa1n06x5               g051(.a(new_n146), .b(new_n135), .c(new_n140), .out0(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n03x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n04x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n03x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand02aa1n04x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nano23aa1n06x5               g059(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n155));
  oa0012aa1n06x5               g060(.a(new_n154), .b(new_n153), .c(new_n149), .o(new_n156));
  nor002aa1n04x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nand42aa1d28x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  aoai13aa1n03x5               g064(.a(new_n159), .b(new_n156), .c(new_n147), .d(new_n155), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n159), .b(new_n156), .c(new_n147), .d(new_n155), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[15] ));
  nor022aa1n04x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nand42aa1n16x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  nona22aa1n02x5               g070(.a(new_n160), .b(new_n165), .c(new_n157), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n165), .o1(new_n167));
  oaoi13aa1n02x7               g072(.a(new_n167), .b(new_n160), .c(\a[15] ), .d(\b[14] ), .o1(new_n168));
  norb02aa1n03x4               g073(.a(new_n166), .b(new_n168), .out0(\s[16] ));
  nano23aa1n06x5               g074(.a(new_n157), .b(new_n163), .c(new_n164), .d(new_n158), .out0(new_n170));
  nano22aa1n03x7               g075(.a(new_n140), .b(new_n155), .c(new_n170), .out0(new_n171));
  oai012aa1n12x5               g076(.a(new_n171), .b(new_n121), .c(new_n111), .o1(new_n172));
  aoai13aa1n12x5               g077(.a(new_n170), .b(new_n156), .c(new_n146), .d(new_n155), .o1(new_n173));
  aoi012aa1n02x7               g078(.a(new_n163), .b(new_n157), .c(new_n164), .o1(new_n174));
  nand23aa1d12x5               g079(.a(new_n172), .b(new_n173), .c(new_n174), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g081(.a(\a[18] ), .o1(new_n177));
  inv040aa1d32x5               g082(.a(\a[17] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[16] ), .o1(new_n179));
  oaoi03aa1n02x5               g084(.a(new_n178), .b(new_n179), .c(new_n175), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[17] ), .c(new_n177), .out0(\s[18] ));
  xroi22aa1d06x4               g086(.a(new_n178), .b(\b[16] ), .c(new_n177), .d(\b[17] ), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n179), .b(new_n178), .o1(new_n183));
  oaoi03aa1n09x5               g088(.a(\a[18] ), .b(\b[17] ), .c(new_n183), .o1(new_n184));
  nor002aa1n20x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  nanb02aa1n02x5               g091(.a(new_n185), .b(new_n186), .out0(new_n187));
  inv000aa1d42x5               g092(.a(new_n187), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g097(.a(new_n185), .o1(new_n193));
  nor002aa1n06x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nand02aa1n03x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  nanp03aa1n03x5               g101(.a(new_n189), .b(new_n193), .c(new_n196), .o1(new_n197));
  tech160nm_fiaoi012aa1n02p5x5 g102(.a(new_n196), .b(new_n189), .c(new_n193), .o1(new_n198));
  norb02aa1n03x4               g103(.a(new_n197), .b(new_n198), .out0(\s[20] ));
  nona23aa1n03x5               g104(.a(new_n195), .b(new_n186), .c(new_n185), .d(new_n194), .out0(new_n200));
  norb02aa1n02x5               g105(.a(new_n182), .b(new_n200), .out0(new_n201));
  oai022aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  oaib12aa1n02x5               g107(.a(new_n202), .b(new_n177), .c(\b[17] ), .out0(new_n203));
  tech160nm_fiaoi012aa1n04x5   g108(.a(new_n194), .b(new_n185), .c(new_n195), .o1(new_n204));
  oai012aa1n06x5               g109(.a(new_n204), .b(new_n200), .c(new_n203), .o1(new_n205));
  xnrc02aa1n12x5               g110(.a(\b[20] ), .b(\a[21] ), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n205), .c(new_n175), .d(new_n201), .o1(new_n208));
  aoi112aa1n02x5               g113(.a(new_n207), .b(new_n205), .c(new_n175), .d(new_n201), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(\s[21] ));
  tech160nm_fixnrc02aa1n04x5   g115(.a(\b[21] ), .b(\a[22] ), .out0(new_n211));
  oai112aa1n02x7               g116(.a(new_n208), .b(new_n211), .c(\b[20] ), .d(\a[21] ), .o1(new_n212));
  oaoi13aa1n02x7               g117(.a(new_n211), .b(new_n208), .c(\a[21] ), .d(\b[20] ), .o1(new_n213));
  norb02aa1n03x4               g118(.a(new_n212), .b(new_n213), .out0(\s[22] ));
  nano23aa1n06x5               g119(.a(new_n185), .b(new_n194), .c(new_n195), .d(new_n186), .out0(new_n215));
  nor042aa1n06x5               g120(.a(new_n211), .b(new_n206), .o1(new_n216));
  and003aa1n03x7               g121(.a(new_n182), .b(new_n216), .c(new_n215), .o(new_n217));
  inv000aa1n02x5               g122(.a(new_n204), .o1(new_n218));
  aoai13aa1n04x5               g123(.a(new_n216), .b(new_n218), .c(new_n215), .d(new_n184), .o1(new_n219));
  inv000aa1d42x5               g124(.a(\a[22] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(\b[21] ), .o1(new_n221));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  oaoi03aa1n09x5               g127(.a(new_n220), .b(new_n221), .c(new_n222), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(new_n219), .b(new_n223), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[22] ), .b(\a[23] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n224), .c(new_n175), .d(new_n217), .o1(new_n227));
  aoi112aa1n02x5               g132(.a(new_n226), .b(new_n224), .c(new_n175), .d(new_n217), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n227), .b(new_n228), .out0(\s[23] ));
  tech160nm_fixnrc02aa1n04x5   g134(.a(\b[23] ), .b(\a[24] ), .out0(new_n230));
  oai112aa1n02x7               g135(.a(new_n227), .b(new_n230), .c(\b[22] ), .d(\a[23] ), .o1(new_n231));
  oaoi13aa1n06x5               g136(.a(new_n230), .b(new_n227), .c(\a[23] ), .d(\b[22] ), .o1(new_n232));
  norb02aa1n03x4               g137(.a(new_n231), .b(new_n232), .out0(\s[24] ));
  nor042aa1n03x5               g138(.a(new_n230), .b(new_n225), .o1(new_n234));
  inv000aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  nano32aa1n02x4               g140(.a(new_n235), .b(new_n182), .c(new_n216), .d(new_n215), .out0(new_n236));
  norp02aa1n02x5               g141(.a(\b[23] ), .b(\a[24] ), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n238));
  norp02aa1n02x5               g143(.a(new_n238), .b(new_n237), .o1(new_n239));
  aoai13aa1n04x5               g144(.a(new_n239), .b(new_n235), .c(new_n219), .d(new_n223), .o1(new_n240));
  aoi012aa1n03x5               g145(.a(new_n240), .b(new_n175), .c(new_n236), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[24] ), .b(\a[25] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnrc02aa1n02x5               g148(.a(new_n241), .b(new_n243), .out0(\s[25] ));
  nor042aa1n03x5               g149(.a(\b[24] ), .b(\a[25] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n243), .b(new_n240), .c(new_n175), .d(new_n236), .o1(new_n247));
  tech160nm_fixnrc02aa1n04x5   g152(.a(\b[25] ), .b(\a[26] ), .out0(new_n248));
  nanp03aa1n03x5               g153(.a(new_n247), .b(new_n246), .c(new_n248), .o1(new_n249));
  tech160nm_fiaoi012aa1n03p5x5 g154(.a(new_n248), .b(new_n247), .c(new_n246), .o1(new_n250));
  norb02aa1n03x4               g155(.a(new_n249), .b(new_n250), .out0(\s[26] ));
  norp02aa1n06x5               g156(.a(new_n248), .b(new_n242), .o1(new_n252));
  inv040aa1n03x5               g157(.a(new_n252), .o1(new_n253));
  nona32aa1n03x5               g158(.a(new_n217), .b(new_n253), .c(new_n230), .d(new_n225), .out0(new_n254));
  inv040aa1n03x5               g159(.a(new_n254), .o1(new_n255));
  nand22aa1n12x5               g160(.a(new_n175), .b(new_n255), .o1(new_n256));
  oao003aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .c(new_n246), .carry(new_n257));
  aobi12aa1n06x5               g162(.a(new_n257), .b(new_n240), .c(new_n252), .out0(new_n258));
  xorc02aa1n12x5               g163(.a(\a[27] ), .b(\b[26] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n256), .out0(\s[27] ));
  norp02aa1n02x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  inv040aa1n03x5               g166(.a(new_n261), .o1(new_n262));
  aobi12aa1n06x5               g167(.a(new_n259), .b(new_n258), .c(new_n256), .out0(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[27] ), .b(\a[28] ), .out0(new_n264));
  nano22aa1n02x4               g169(.a(new_n263), .b(new_n262), .c(new_n264), .out0(new_n265));
  aoi013aa1n03x5               g170(.a(new_n254), .b(new_n172), .c(new_n173), .d(new_n174), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n223), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n234), .b(new_n267), .c(new_n205), .d(new_n216), .o1(new_n268));
  aoai13aa1n03x5               g173(.a(new_n257), .b(new_n253), .c(new_n268), .d(new_n239), .o1(new_n269));
  oaih12aa1n02x5               g174(.a(new_n259), .b(new_n269), .c(new_n266), .o1(new_n270));
  tech160nm_fiaoi012aa1n02p5x5 g175(.a(new_n264), .b(new_n270), .c(new_n262), .o1(new_n271));
  nor002aa1n02x5               g176(.a(new_n271), .b(new_n265), .o1(\s[28] ));
  norb02aa1n02x5               g177(.a(new_n259), .b(new_n264), .out0(new_n273));
  oaih12aa1n02x5               g178(.a(new_n273), .b(new_n269), .c(new_n266), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .carry(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[28] ), .b(\a[29] ), .out0(new_n276));
  tech160nm_fiaoi012aa1n02p5x5 g181(.a(new_n276), .b(new_n274), .c(new_n275), .o1(new_n277));
  aobi12aa1n03x5               g182(.a(new_n273), .b(new_n258), .c(new_n256), .out0(new_n278));
  nano22aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n276), .out0(new_n279));
  norp02aa1n03x5               g184(.a(new_n277), .b(new_n279), .o1(\s[29] ));
  xorb03aa1n02x5               g185(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g186(.a(new_n259), .b(new_n276), .c(new_n264), .out0(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n269), .c(new_n266), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[29] ), .b(\b[28] ), .c(new_n275), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[29] ), .b(\a[30] ), .out0(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  aobi12aa1n06x5               g191(.a(new_n282), .b(new_n258), .c(new_n256), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n284), .c(new_n285), .out0(new_n288));
  nor002aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[30] ));
  norb02aa1n02x5               g194(.a(new_n282), .b(new_n285), .out0(new_n290));
  oaih12aa1n02x5               g195(.a(new_n290), .b(new_n269), .c(new_n266), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[30] ), .b(\b[29] ), .c(new_n284), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[30] ), .b(\a[31] ), .out0(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n03x5               g199(.a(new_n290), .b(new_n258), .c(new_n256), .out0(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[31] ));
  oai012aa1n02x5               g202(.a(new_n114), .b(new_n115), .c(new_n113), .o1(new_n298));
  xnrb03aa1n02x5               g203(.a(new_n298), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g204(.a(\a[3] ), .b(\b[2] ), .c(new_n298), .o1(new_n300));
  xorb03aa1n02x5               g205(.a(new_n300), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorc02aa1n02x5               g206(.a(\a[5] ), .b(\b[4] ), .out0(new_n302));
  aoi112aa1n02x5               g207(.a(new_n302), .b(new_n112), .c(new_n117), .d(new_n118), .o1(new_n303));
  aoai13aa1n02x5               g208(.a(new_n302), .b(new_n112), .c(new_n117), .d(new_n118), .o1(new_n304));
  nanb02aa1n02x5               g209(.a(new_n303), .b(new_n304), .out0(\s[5] ));
  nanp02aa1n02x5               g210(.a(new_n304), .b(new_n119), .o1(new_n306));
  xnrb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g212(.a(\a[6] ), .b(\b[5] ), .c(new_n306), .carry(new_n308));
  xnbna2aa1n03x5               g213(.a(new_n308), .b(new_n109), .c(new_n105), .out0(\s[7] ));
  oaoi03aa1n02x5               g214(.a(\a[7] ), .b(\b[6] ), .c(new_n308), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g216(.a(new_n135), .b(\b[8] ), .c(new_n100), .out0(\s[9] ));
endmodule


