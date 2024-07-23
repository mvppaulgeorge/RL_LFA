// Benchmark "adder" written by ABC on Wed Jul 17 16:33:14 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n311,
    new_n313, new_n315, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n09x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n09x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nano23aa1n09x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .d(new_n98), .out0(new_n102));
  and002aa1n02x7               g007(.a(\b[5] ), .b(\a[6] ), .o(new_n103));
  norp02aa1n24x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  oab012aa1n03x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .out0(new_n106));
  oai022aa1n02x5               g011(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n107));
  aoi022aa1n09x5               g012(.a(new_n102), .b(new_n106), .c(new_n98), .d(new_n107), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\a[2] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[1] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  nand22aa1n12x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aob012aa1n06x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .out0(new_n114));
  nor002aa1n04x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nanp02aa1n12x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  norb02aa1n06x4               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nor042aa1n03x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nand42aa1n04x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  norb02aa1n02x5               g024(.a(new_n119), .b(new_n118), .out0(new_n120));
  nand03aa1n04x5               g025(.a(new_n114), .b(new_n117), .c(new_n120), .o1(new_n121));
  tech160nm_fioai012aa1n03p5x5 g026(.a(new_n116), .b(new_n118), .c(new_n115), .o1(new_n122));
  nand02aa1n03x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nand22aa1n04x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  nano23aa1n02x5               g029(.a(new_n105), .b(new_n104), .c(new_n124), .d(new_n123), .out0(new_n125));
  nand02aa1n02x5               g030(.a(new_n125), .b(new_n102), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n108), .b(new_n126), .c(new_n121), .d(new_n122), .o1(new_n127));
  nand42aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  aoi012aa1n02x5               g033(.a(new_n97), .b(new_n127), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n04x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nano23aa1n03x7               g037(.a(new_n97), .b(new_n131), .c(new_n132), .d(new_n128), .out0(new_n133));
  tech160nm_fioai012aa1n05x5   g038(.a(new_n132), .b(new_n131), .c(new_n97), .o1(new_n134));
  aob012aa1n03x5               g039(.a(new_n134), .b(new_n127), .c(new_n133), .out0(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand02aa1n08x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  tech160nm_fiaoi012aa1n05x5   g043(.a(new_n137), .b(new_n135), .c(new_n138), .o1(new_n139));
  xnrb03aa1n03x5               g044(.a(new_n139), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n16x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nona23aa1n09x5               g047(.a(new_n142), .b(new_n138), .c(new_n137), .d(new_n141), .out0(new_n143));
  tech160nm_fioai012aa1n04x5   g048(.a(new_n142), .b(new_n141), .c(new_n137), .o1(new_n144));
  oaih12aa1n06x5               g049(.a(new_n144), .b(new_n143), .c(new_n134), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nano23aa1n06x5               g051(.a(new_n137), .b(new_n141), .c(new_n142), .d(new_n138), .out0(new_n147));
  nand23aa1n03x5               g052(.a(new_n127), .b(new_n133), .c(new_n147), .o1(new_n148));
  nanp02aa1n06x5               g053(.a(new_n148), .b(new_n146), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g055(.a(\a[14] ), .o1(new_n151));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand02aa1d28x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  tech160nm_fiaoi012aa1n05x5   g058(.a(new_n152), .b(new_n149), .c(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(new_n151), .out0(\s[14] ));
  nor002aa1n20x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand02aa1d16x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1d15x5               g062(.a(new_n152), .b(new_n156), .c(new_n157), .d(new_n153), .out0(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n156), .b(new_n152), .c(new_n157), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n159), .c(new_n148), .d(new_n146), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  tech160nm_finand02aa1n05x5   g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  nor042aa1n04x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nand42aa1n06x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n163), .c(new_n161), .d(new_n165), .o1(new_n169));
  nand42aa1n02x5               g074(.a(new_n161), .b(new_n165), .o1(new_n170));
  nona22aa1n03x5               g075(.a(new_n170), .b(new_n168), .c(new_n163), .out0(new_n171));
  nanp02aa1n03x5               g076(.a(new_n171), .b(new_n169), .o1(\s[16] ));
  nano23aa1n09x5               g077(.a(new_n163), .b(new_n166), .c(new_n167), .d(new_n164), .out0(new_n173));
  oa0012aa1n12x5               g078(.a(new_n157), .b(new_n156), .c(new_n152), .o(new_n174));
  tech160nm_fioai012aa1n03p5x5 g079(.a(new_n167), .b(new_n166), .c(new_n163), .o1(new_n175));
  aobi12aa1d24x5               g080(.a(new_n175), .b(new_n173), .c(new_n174), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  nand22aa1n03x5               g082(.a(new_n173), .b(new_n158), .o1(new_n178));
  aoib12aa1n09x5               g083(.a(new_n177), .b(new_n145), .c(new_n178), .out0(new_n179));
  nano22aa1n03x7               g084(.a(new_n178), .b(new_n133), .c(new_n147), .out0(new_n180));
  nand02aa1d06x5               g085(.a(new_n127), .b(new_n180), .o1(new_n181));
  xorc02aa1n02x5               g086(.a(\a[17] ), .b(\b[16] ), .out0(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n181), .c(new_n179), .out0(\s[17] ));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(\b[16] ), .b(new_n184), .out0(new_n185));
  nanb02aa1n02x5               g090(.a(new_n134), .b(new_n147), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n176), .b(new_n178), .c(new_n186), .d(new_n144), .o1(new_n187));
  aoai13aa1n02x5               g092(.a(new_n182), .b(new_n187), .c(new_n127), .d(new_n180), .o1(new_n188));
  xnrc02aa1n02x5               g093(.a(\b[17] ), .b(\a[18] ), .out0(new_n189));
  xobna2aa1n03x5               g094(.a(new_n189), .b(new_n188), .c(new_n185), .out0(\s[18] ));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  xroi22aa1d04x5               g096(.a(new_n184), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n187), .c(new_n127), .d(new_n180), .o1(new_n193));
  oai022aa1n04x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n12x5               g099(.a(new_n194), .b(new_n191), .c(\b[17] ), .out0(new_n195));
  nor022aa1n12x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nand22aa1n03x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n193), .c(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n06x5               g106(.a(new_n193), .b(new_n195), .o1(new_n202));
  nor022aa1n12x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand22aa1n03x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  aoai13aa1n03x5               g110(.a(new_n205), .b(new_n196), .c(new_n202), .d(new_n199), .o1(new_n206));
  nand22aa1n03x5               g111(.a(new_n202), .b(new_n199), .o1(new_n207));
  nona22aa1n02x4               g112(.a(new_n207), .b(new_n205), .c(new_n196), .out0(new_n208));
  nanp02aa1n03x5               g113(.a(new_n208), .b(new_n206), .o1(\s[20] ));
  nona23aa1n09x5               g114(.a(new_n204), .b(new_n197), .c(new_n196), .d(new_n203), .out0(new_n210));
  tech160nm_fioai012aa1n03p5x5 g115(.a(new_n204), .b(new_n203), .c(new_n196), .o1(new_n211));
  oai012aa1d24x5               g116(.a(new_n211), .b(new_n210), .c(new_n195), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nano23aa1n06x5               g118(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n214));
  nanb03aa1n02x5               g119(.a(new_n189), .b(new_n214), .c(new_n182), .out0(new_n215));
  aoai13aa1n06x5               g120(.a(new_n213), .b(new_n215), .c(new_n181), .d(new_n179), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xnrc02aa1n12x5               g123(.a(\b[20] ), .b(\a[21] ), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  tech160nm_fixnrc02aa1n05x5   g125(.a(\b[21] ), .b(\a[22] ), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n218), .c(new_n216), .d(new_n220), .o1(new_n222));
  nand02aa1n03x5               g127(.a(new_n216), .b(new_n220), .o1(new_n223));
  nona22aa1n03x5               g128(.a(new_n223), .b(new_n221), .c(new_n218), .out0(new_n224));
  nanp02aa1n03x5               g129(.a(new_n224), .b(new_n222), .o1(\s[22] ));
  nor042aa1n02x5               g130(.a(new_n221), .b(new_n219), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\a[22] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[21] ), .o1(new_n228));
  oao003aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n218), .carry(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n212), .c(new_n226), .o1(new_n230));
  nand23aa1n03x5               g135(.a(new_n192), .b(new_n226), .c(new_n214), .o1(new_n231));
  aoai13aa1n06x5               g136(.a(new_n230), .b(new_n231), .c(new_n181), .d(new_n179), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  tech160nm_fixnrc02aa1n04x5   g140(.a(\b[23] ), .b(\a[24] ), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n237));
  nand42aa1n02x5               g142(.a(new_n232), .b(new_n235), .o1(new_n238));
  nona22aa1n02x4               g143(.a(new_n238), .b(new_n236), .c(new_n234), .out0(new_n239));
  nanp02aa1n03x5               g144(.a(new_n239), .b(new_n237), .o1(\s[24] ));
  norb02aa1n09x5               g145(.a(new_n235), .b(new_n236), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  nano32aa1n02x4               g147(.a(new_n242), .b(new_n192), .c(new_n226), .d(new_n214), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n187), .c(new_n127), .d(new_n180), .o1(new_n244));
  oaoi03aa1n02x5               g149(.a(\a[18] ), .b(\b[17] ), .c(new_n185), .o1(new_n245));
  inv040aa1n03x5               g150(.a(new_n211), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n226), .b(new_n246), .c(new_n214), .d(new_n245), .o1(new_n247));
  inv000aa1n02x5               g152(.a(new_n229), .o1(new_n248));
  orn002aa1n03x5               g153(.a(\a[23] ), .b(\b[22] ), .o(new_n249));
  oaoi03aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n249), .o1(new_n250));
  inv030aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n242), .c(new_n247), .d(new_n248), .o1(new_n252));
  nanb02aa1n06x5               g157(.a(new_n252), .b(new_n244), .out0(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  tech160nm_fixorc02aa1n03p5x5 g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  xorc02aa1n12x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n255), .c(new_n253), .d(new_n256), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n253), .b(new_n256), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n258), .c(new_n255), .out0(new_n261));
  nanp02aa1n03x5               g166(.a(new_n261), .b(new_n259), .o1(\s[26] ));
  and002aa1n06x5               g167(.a(new_n257), .b(new_n256), .o(new_n263));
  nand02aa1n02x5               g168(.a(new_n252), .b(new_n263), .o1(new_n264));
  oai022aa1n02x5               g169(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n265));
  aob012aa1n02x5               g170(.a(new_n265), .b(\b[25] ), .c(\a[26] ), .out0(new_n266));
  nano22aa1n03x7               g171(.a(new_n231), .b(new_n263), .c(new_n241), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n187), .c(new_n127), .d(new_n180), .o1(new_n268));
  nanp03aa1n06x5               g173(.a(new_n264), .b(new_n268), .c(new_n266), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  aoai13aa1n02x7               g178(.a(new_n273), .b(new_n271), .c(new_n269), .d(new_n272), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n241), .b(new_n229), .c(new_n212), .d(new_n226), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n263), .o1(new_n276));
  aoai13aa1n04x5               g181(.a(new_n266), .b(new_n276), .c(new_n275), .d(new_n251), .o1(new_n277));
  aobi12aa1n09x5               g182(.a(new_n267), .b(new_n181), .c(new_n179), .out0(new_n278));
  oaih12aa1n02x5               g183(.a(new_n272), .b(new_n277), .c(new_n278), .o1(new_n279));
  nona22aa1n02x4               g184(.a(new_n279), .b(new_n273), .c(new_n271), .out0(new_n280));
  nanp02aa1n03x5               g185(.a(new_n274), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n272), .b(new_n273), .out0(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n277), .c(new_n278), .o1(new_n283));
  inv000aa1n03x5               g188(.a(new_n271), .o1(new_n284));
  oaoi03aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nona22aa1n03x5               g191(.a(new_n283), .b(new_n285), .c(new_n286), .out0(new_n287));
  aoai13aa1n02x7               g192(.a(new_n286), .b(new_n285), .c(new_n269), .d(new_n282), .o1(new_n288));
  nanp02aa1n03x5               g193(.a(new_n288), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n272), .b(new_n286), .c(new_n273), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .o1(new_n293));
  tech160nm_fixorc02aa1n02p5x5 g198(.a(\a[30] ), .b(\b[29] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  aoai13aa1n02x7               g200(.a(new_n295), .b(new_n293), .c(new_n269), .d(new_n291), .o1(new_n296));
  oaih12aa1n02x5               g201(.a(new_n291), .b(new_n277), .c(new_n278), .o1(new_n297));
  nona22aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n295), .out0(new_n298));
  nanp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  nanp02aa1n02x5               g204(.a(new_n293), .b(new_n294), .o1(new_n300));
  oai012aa1n02x5               g205(.a(new_n300), .b(\b[29] ), .c(\a[30] ), .o1(new_n301));
  nano23aa1n06x5               g206(.a(new_n286), .b(new_n273), .c(new_n294), .d(new_n272), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n277), .c(new_n278), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nona22aa1n03x5               g209(.a(new_n303), .b(new_n304), .c(new_n301), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n304), .b(new_n301), .c(new_n269), .d(new_n302), .o1(new_n306));
  nanp02aa1n03x5               g211(.a(new_n306), .b(new_n305), .o1(\s[31] ));
  xorb03aa1n02x5               g212(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g213(.a(new_n118), .b(new_n114), .c(new_n119), .o1(new_n309));
  xnrc02aa1n02x5               g214(.a(new_n309), .b(new_n117), .out0(\s[4] ));
  nanp02aa1n02x5               g215(.a(new_n121), .b(new_n122), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g217(.a(new_n105), .b(new_n311), .c(new_n124), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g219(.a(new_n106), .b(new_n311), .c(new_n125), .o(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g221(.a(new_n100), .b(new_n315), .c(new_n101), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


