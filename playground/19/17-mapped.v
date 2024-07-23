// Benchmark "adder" written by ABC on Wed Jul 17 21:49:37 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n319, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1n06x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n03x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor042aa1n04x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor022aa1n06x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  tech160nm_finand02aa1n03p5x5 g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand02aa1n03x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nor042aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  oai012aa1n04x7               g013(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n109));
  tech160nm_fiaoi012aa1n03p5x5 g014(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n110));
  oai012aa1n12x5               g015(.a(new_n110), .b(new_n105), .c(new_n109), .o1(new_n111));
  norp02aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  tech160nm_finand02aa1n03p5x5 g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  norp03aa1d12x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[5] ), .o1(new_n121));
  aoi112aa1n03x5               g026(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n120), .c(new_n121), .o1(new_n123));
  oai012aa1n02x7               g028(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n124));
  oai012aa1n06x5               g029(.a(new_n124), .b(new_n116), .c(new_n123), .o1(new_n125));
  xnrc02aa1n12x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n128));
  oaib12aa1n02x5               g033(.a(new_n99), .b(new_n100), .c(new_n128), .out0(new_n129));
  nona22aa1n02x4               g034(.a(new_n128), .b(new_n100), .c(new_n99), .out0(new_n130));
  nanp02aa1n02x5               g035(.a(new_n129), .b(new_n130), .o1(\s[10] ));
  nor002aa1n10x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n08x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xobna2aa1n03x5               g039(.a(new_n134), .b(new_n130), .c(new_n98), .out0(\s[11] ));
  inv030aa1n02x5               g040(.a(new_n132), .o1(new_n136));
  nanp03aa1n03x5               g041(.a(new_n130), .b(new_n98), .c(new_n134), .o1(new_n137));
  nor022aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n136), .out0(\s[12] ));
  tech160nm_fiaoi012aa1n05x5   g046(.a(new_n125), .b(new_n111), .c(new_n119), .o1(new_n142));
  nona23aa1n03x5               g047(.a(new_n139), .b(new_n133), .c(new_n132), .d(new_n138), .out0(new_n143));
  aoi012aa1n12x5               g048(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n144));
  oaoi03aa1n09x5               g049(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n145));
  oabi12aa1n03x5               g050(.a(new_n145), .b(new_n143), .c(new_n144), .out0(new_n146));
  nano23aa1n06x5               g051(.a(new_n132), .b(new_n138), .c(new_n139), .d(new_n133), .out0(new_n147));
  nona22aa1n09x5               g052(.a(new_n147), .b(new_n126), .c(new_n99), .out0(new_n148));
  oabi12aa1n06x5               g053(.a(new_n146), .b(new_n142), .c(new_n148), .out0(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n06x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  tech160nm_finand02aa1n03p5x5 g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n149), .c(new_n152), .o1(new_n153));
  xnrb03aa1n03x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n20x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  nand42aa1d28x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  nor022aa1n06x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand42aa1n03x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1n09x5               g065(.a(new_n151), .b(new_n159), .c(new_n160), .d(new_n152), .out0(new_n161));
  oai012aa1n02x5               g066(.a(new_n160), .b(new_n159), .c(new_n151), .o1(new_n162));
  inv000aa1n02x5               g067(.a(new_n162), .o1(new_n163));
  aoai13aa1n06x5               g068(.a(new_n158), .b(new_n163), .c(new_n149), .d(new_n161), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n158), .b(new_n163), .c(new_n149), .d(new_n161), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[15] ));
  inv000aa1d42x5               g071(.a(new_n155), .o1(new_n167));
  nor042aa1n04x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand42aa1n20x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  nanp03aa1n03x5               g075(.a(new_n164), .b(new_n167), .c(new_n170), .o1(new_n171));
  tech160nm_fiaoi012aa1n03p5x5 g076(.a(new_n170), .b(new_n164), .c(new_n167), .o1(new_n172));
  norb02aa1n03x4               g077(.a(new_n171), .b(new_n172), .out0(\s[16] ));
  nano23aa1d15x5               g078(.a(new_n155), .b(new_n168), .c(new_n169), .d(new_n156), .out0(new_n174));
  nano22aa1d15x5               g079(.a(new_n148), .b(new_n161), .c(new_n174), .out0(new_n175));
  aoai13aa1n12x5               g080(.a(new_n175), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n176));
  aoai13aa1n06x5               g081(.a(new_n174), .b(new_n163), .c(new_n146), .d(new_n161), .o1(new_n177));
  oai012aa1n06x5               g082(.a(new_n169), .b(new_n168), .c(new_n155), .o1(new_n178));
  nand23aa1d12x5               g083(.a(new_n176), .b(new_n177), .c(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n09x5               g085(.a(\b[16] ), .b(\a[17] ), .o1(new_n181));
  nand42aa1n10x5               g086(.a(\b[16] ), .b(\a[17] ), .o1(new_n182));
  tech160nm_fiaoi012aa1n05x5   g087(.a(new_n181), .b(new_n179), .c(new_n182), .o1(new_n183));
  xnrb03aa1n03x5               g088(.a(new_n183), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1n02x5               g089(.a(new_n144), .o1(new_n185));
  aoai13aa1n06x5               g090(.a(new_n161), .b(new_n145), .c(new_n147), .d(new_n185), .o1(new_n186));
  nand02aa1n02x5               g091(.a(new_n186), .b(new_n162), .o1(new_n187));
  aobi12aa1n06x5               g092(.a(new_n178), .b(new_n187), .c(new_n174), .out0(new_n188));
  nor042aa1n06x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nand02aa1n16x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nano23aa1d15x5               g095(.a(new_n181), .b(new_n189), .c(new_n190), .d(new_n182), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  aoi012aa1d18x5               g097(.a(new_n189), .b(new_n181), .c(new_n190), .o1(new_n193));
  aoai13aa1n04x5               g098(.a(new_n193), .b(new_n192), .c(new_n188), .d(new_n176), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand22aa1n06x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nor002aa1n12x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand22aa1n04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoi112aa1n02x5               g107(.a(new_n197), .b(new_n202), .c(new_n194), .d(new_n199), .o1(new_n203));
  inv040aa1n08x5               g108(.a(new_n197), .o1(new_n204));
  inv000aa1n06x5               g109(.a(new_n193), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n199), .b(new_n205), .c(new_n179), .d(new_n191), .o1(new_n206));
  aobi12aa1n06x5               g111(.a(new_n202), .b(new_n206), .c(new_n204), .out0(new_n207));
  nor002aa1n02x5               g112(.a(new_n207), .b(new_n203), .o1(\s[20] ));
  nano23aa1n06x5               g113(.a(new_n197), .b(new_n200), .c(new_n201), .d(new_n198), .out0(new_n209));
  nand22aa1n06x5               g114(.a(new_n209), .b(new_n191), .o1(new_n210));
  nona23aa1d18x5               g115(.a(new_n201), .b(new_n198), .c(new_n197), .d(new_n200), .out0(new_n211));
  oaoi03aa1n12x5               g116(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .o1(new_n212));
  inv040aa1n02x5               g117(.a(new_n212), .o1(new_n213));
  oai012aa1d24x5               g118(.a(new_n213), .b(new_n211), .c(new_n193), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n04x5               g120(.a(new_n215), .b(new_n210), .c(new_n188), .d(new_n176), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  inv000aa1n09x5               g126(.a(new_n218), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n210), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n219), .b(new_n214), .c(new_n179), .d(new_n223), .o1(new_n224));
  aobi12aa1n06x5               g129(.a(new_n220), .b(new_n224), .c(new_n222), .out0(new_n225));
  nor002aa1n02x5               g130(.a(new_n225), .b(new_n221), .o1(\s[22] ));
  nano22aa1n03x7               g131(.a(new_n210), .b(new_n219), .c(new_n220), .out0(new_n227));
  inv000aa1n02x5               g132(.a(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\a[21] ), .o1(new_n229));
  inv020aa1n04x5               g134(.a(\a[22] ), .o1(new_n230));
  xroi22aa1d06x4               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  oaoi03aa1n12x5               g136(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .o1(new_n232));
  aoi012aa1d18x5               g137(.a(new_n232), .b(new_n214), .c(new_n231), .o1(new_n233));
  aoai13aa1n04x5               g138(.a(new_n233), .b(new_n228), .c(new_n188), .d(new_n176), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  tech160nm_fixorc02aa1n05x5   g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  aoi112aa1n02x5               g143(.a(new_n236), .b(new_n238), .c(new_n234), .d(new_n237), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n236), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n233), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n237), .b(new_n241), .c(new_n179), .d(new_n227), .o1(new_n242));
  aobi12aa1n06x5               g147(.a(new_n238), .b(new_n242), .c(new_n240), .out0(new_n243));
  nor042aa1n03x5               g148(.a(new_n243), .b(new_n239), .o1(\s[24] ));
  and002aa1n02x5               g149(.a(new_n238), .b(new_n237), .o(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  nano32aa1n02x4               g151(.a(new_n246), .b(new_n231), .c(new_n209), .d(new_n191), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n231), .b(new_n212), .c(new_n209), .d(new_n205), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n232), .o1(new_n250));
  nanp02aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  oai022aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n252), .b(new_n251), .o1(new_n253));
  aoai13aa1n12x5               g158(.a(new_n253), .b(new_n246), .c(new_n249), .d(new_n250), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n248), .c(new_n188), .d(new_n176), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  tech160nm_fixorc02aa1n05x5   g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  xorc02aa1n03x5               g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  aoi112aa1n03x4               g165(.a(new_n258), .b(new_n260), .c(new_n256), .d(new_n259), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n258), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n259), .b(new_n254), .c(new_n179), .d(new_n247), .o1(new_n263));
  aobi12aa1n06x5               g168(.a(new_n260), .b(new_n263), .c(new_n262), .out0(new_n264));
  nor002aa1n02x5               g169(.a(new_n264), .b(new_n261), .o1(\s[26] ));
  inv030aa1n02x5               g170(.a(new_n142), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n174), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n178), .b(new_n267), .c(new_n186), .d(new_n162), .o1(new_n268));
  and002aa1n02x5               g173(.a(new_n260), .b(new_n259), .o(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  nano23aa1n06x5               g175(.a(new_n210), .b(new_n270), .c(new_n245), .d(new_n231), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n268), .c(new_n266), .d(new_n175), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n273));
  aobi12aa1n06x5               g178(.a(new_n273), .b(new_n254), .c(new_n269), .out0(new_n274));
  xorc02aa1n12x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n272), .c(new_n274), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  aobi12aa1n02x7               g183(.a(new_n275), .b(new_n272), .c(new_n274), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n03x5               g185(.a(new_n279), .b(new_n278), .c(new_n280), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n245), .b(new_n232), .c(new_n214), .d(new_n231), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n273), .b(new_n270), .c(new_n282), .d(new_n253), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n275), .b(new_n283), .c(new_n179), .d(new_n271), .o1(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n280), .b(new_n284), .c(new_n278), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n275), .b(new_n280), .out0(new_n287));
  aobi12aa1n02x7               g192(.a(new_n287), .b(new_n272), .c(new_n274), .out0(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n287), .b(new_n283), .c(new_n179), .d(new_n271), .o1(new_n292));
  tech160nm_fiaoi012aa1n03p5x5 g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  nor042aa1n03x5               g198(.a(new_n293), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n275), .b(new_n290), .c(new_n280), .out0(new_n296));
  aobi12aa1n02x7               g201(.a(new_n296), .b(new_n272), .c(new_n274), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n296), .b(new_n283), .c(new_n179), .d(new_n271), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g208(.a(new_n296), .b(new_n299), .out0(new_n304));
  aobi12aa1n02x7               g209(.a(new_n304), .b(new_n272), .c(new_n274), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n304), .b(new_n283), .c(new_n179), .d(new_n271), .o1(new_n309));
  tech160nm_fiaoi012aa1n03p5x5 g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  nor042aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g220(.a(new_n118), .b(new_n111), .out0(new_n316));
  tech160nm_fioai012aa1n03p5x5 g221(.a(new_n316), .b(\b[4] ), .c(\a[5] ), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g223(.a(new_n114), .o1(new_n319));
  tech160nm_fioaoi03aa1n03p5x5 g224(.a(new_n120), .b(new_n121), .c(new_n317), .o1(new_n320));
  xnbna2aa1n03x5               g225(.a(new_n320), .b(new_n319), .c(new_n115), .out0(\s[7] ));
  oaoi03aa1n03x5               g226(.a(\a[7] ), .b(\b[6] ), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g228(.a(new_n142), .b(new_n127), .out0(\s[9] ));
endmodule


