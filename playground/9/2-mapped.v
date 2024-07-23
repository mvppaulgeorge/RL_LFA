// Benchmark "adder" written by ABC on Wed Jul 17 16:31:59 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n03x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[8] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[7] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  oaoi03aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .out0(new_n102));
  nor022aa1n04x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nor022aa1n04x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nand42aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  aoi012aa1n03x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  tech160nm_fixnrc02aa1n04x5   g011(.a(\b[6] ), .b(\a[7] ), .out0(new_n107));
  norp03aa1n02x5               g012(.a(new_n107), .b(new_n102), .c(new_n106), .o1(new_n108));
  norb02aa1n02x5               g013(.a(new_n101), .b(new_n108), .out0(new_n109));
  inv040aa1d32x5               g014(.a(\a[4] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\b[3] ), .o1(new_n111));
  nand42aa1n02x5               g016(.a(new_n111), .b(new_n110), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(new_n112), .b(new_n113), .o1(new_n114));
  norp02aa1n12x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  oaoi03aa1n02x5               g020(.a(new_n110), .b(new_n111), .c(new_n115), .o1(new_n116));
  nor042aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nand22aa1n06x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nand22aa1n04x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  aoi012aa1n12x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  nand42aa1n03x5               g025(.a(\b[2] ), .b(\a[3] ), .o1(new_n121));
  nanb02aa1n06x5               g026(.a(new_n115), .b(new_n121), .out0(new_n122));
  oai013aa1n06x5               g027(.a(new_n116), .b(new_n120), .c(new_n122), .d(new_n114), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  nona23aa1n03x5               g029(.a(new_n105), .b(new_n124), .c(new_n104), .d(new_n103), .out0(new_n125));
  nor043aa1n02x5               g030(.a(new_n125), .b(new_n107), .c(new_n102), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(new_n123), .b(new_n126), .o1(new_n127));
  nand42aa1n02x5               g032(.a(new_n127), .b(new_n109), .o1(new_n128));
  nand42aa1n03x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  aoi012aa1n02x5               g034(.a(new_n97), .b(new_n128), .c(new_n129), .o1(new_n130));
  xnrb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand02aa1n03x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  aoi012aa1n06x5               g038(.a(new_n132), .b(new_n97), .c(new_n133), .o1(new_n134));
  nona23aa1n02x4               g039(.a(new_n133), .b(new_n129), .c(new_n97), .d(new_n132), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n134), .b(new_n135), .c(new_n127), .d(new_n109), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(\a[12] ), .o1(new_n138));
  norp02aa1n06x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  tech160nm_finand02aa1n03p5x5 g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  aoi012aa1n02x5               g045(.a(new_n139), .b(new_n136), .c(new_n140), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(new_n138), .out0(\s[12] ));
  nor022aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  ao0012aa1n03x5               g049(.a(new_n143), .b(new_n139), .c(new_n144), .o(new_n145));
  nona23aa1n09x5               g050(.a(new_n144), .b(new_n140), .c(new_n139), .d(new_n143), .out0(new_n146));
  oabi12aa1n18x5               g051(.a(new_n145), .b(new_n146), .c(new_n134), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nano23aa1n03x7               g053(.a(new_n97), .b(new_n132), .c(new_n133), .d(new_n129), .out0(new_n149));
  nano23aa1n02x5               g054(.a(new_n139), .b(new_n143), .c(new_n144), .d(new_n140), .out0(new_n150));
  nanp02aa1n03x5               g055(.a(new_n150), .b(new_n149), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n148), .b(new_n151), .c(new_n127), .d(new_n109), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g058(.a(\a[14] ), .o1(new_n154));
  nor042aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n152), .c(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(new_n154), .out0(\s[14] ));
  norp02aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n155), .c(new_n160), .o1(new_n161));
  nano23aa1n06x5               g066(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n162));
  aob012aa1n02x5               g067(.a(new_n161), .b(new_n147), .c(new_n162), .out0(new_n163));
  nona23aa1n03x5               g068(.a(new_n160), .b(new_n156), .c(new_n155), .d(new_n159), .out0(new_n164));
  norp03aa1n02x5               g069(.a(new_n164), .b(new_n146), .c(new_n135), .o1(new_n165));
  nor002aa1d32x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1n04x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanb02aa1n12x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n163), .c(new_n128), .d(new_n165), .o1(new_n170));
  aoi112aa1n02x5               g075(.a(new_n163), .b(new_n169), .c(new_n128), .d(new_n165), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(\s[15] ));
  inv000aa1d42x5               g077(.a(new_n166), .o1(new_n173));
  nor022aa1n06x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n03x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  xobna2aa1n03x5               g081(.a(new_n176), .b(new_n170), .c(new_n173), .out0(\s[16] ));
  aoi012aa1n02x5               g082(.a(new_n174), .b(new_n166), .c(new_n175), .o1(new_n178));
  oai013aa1n03x5               g083(.a(new_n178), .b(new_n161), .c(new_n168), .d(new_n176), .o1(new_n179));
  nor043aa1n02x5               g084(.a(new_n164), .b(new_n168), .c(new_n176), .o1(new_n180));
  aoi012aa1d18x5               g085(.a(new_n179), .b(new_n147), .c(new_n180), .o1(new_n181));
  oai013aa1n03x4               g086(.a(new_n101), .b(new_n102), .c(new_n107), .d(new_n106), .o1(new_n182));
  nano23aa1n02x5               g087(.a(new_n166), .b(new_n174), .c(new_n175), .d(new_n167), .out0(new_n183));
  nano22aa1n03x7               g088(.a(new_n151), .b(new_n162), .c(new_n183), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n182), .c(new_n123), .d(new_n126), .o1(new_n185));
  xorc02aa1n12x5               g090(.a(\a[17] ), .b(\b[16] ), .out0(new_n186));
  xnbna2aa1n03x5               g091(.a(new_n186), .b(new_n185), .c(new_n181), .out0(\s[17] ));
  nanp02aa1n06x5               g092(.a(new_n185), .b(new_n181), .o1(new_n188));
  nor042aa1n06x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  tech160nm_fiaoi012aa1n05x5   g094(.a(new_n189), .b(new_n188), .c(new_n186), .o1(new_n190));
  inv040aa1d32x5               g095(.a(\a[18] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[17] ), .o1(new_n192));
  nand42aa1n06x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nand22aa1n12x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n190), .b(new_n194), .c(new_n193), .out0(\s[18] ));
  aob012aa1d24x5               g100(.a(new_n193), .b(new_n189), .c(new_n194), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  inv000aa1n06x5               g102(.a(new_n186), .o1(new_n198));
  nano22aa1n12x5               g103(.a(new_n198), .b(new_n193), .c(new_n194), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n06x5               g105(.a(new_n197), .b(new_n200), .c(new_n185), .d(new_n181), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n12x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand42aa1n02x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nor002aa1n12x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1n10x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n12x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n204), .c(new_n201), .d(new_n205), .o1(new_n210));
  norb02aa1n06x4               g115(.a(new_n205), .b(new_n204), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(new_n201), .b(new_n211), .o1(new_n212));
  nona22aa1n02x4               g117(.a(new_n212), .b(new_n209), .c(new_n204), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n213), .b(new_n210), .o1(\s[20] ));
  aoi012aa1d24x5               g119(.a(new_n206), .b(new_n204), .c(new_n207), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  nano23aa1n06x5               g121(.a(new_n204), .b(new_n206), .c(new_n207), .d(new_n205), .out0(new_n217));
  aoi012aa1n12x5               g122(.a(new_n216), .b(new_n217), .c(new_n196), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n199), .b(new_n217), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n218), .b(new_n219), .c(new_n185), .d(new_n181), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nand02aa1d08x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  nor022aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .o1(new_n225));
  nand02aa1d08x5               g130(.a(\b[21] ), .b(\a[22] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n222), .c(new_n220), .d(new_n224), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n220), .b(new_n224), .o1(new_n230));
  nona22aa1n02x4               g135(.a(new_n230), .b(new_n228), .c(new_n222), .out0(new_n231));
  nanp02aa1n02x5               g136(.a(new_n231), .b(new_n229), .o1(\s[22] ));
  inv000aa1d42x5               g137(.a(new_n218), .o1(new_n233));
  aoi012aa1n06x5               g138(.a(new_n225), .b(new_n222), .c(new_n226), .o1(new_n234));
  inv020aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  nano23aa1d15x5               g140(.a(new_n222), .b(new_n225), .c(new_n226), .d(new_n223), .out0(new_n236));
  aoi012aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n236), .o1(new_n237));
  nano22aa1n02x4               g142(.a(new_n200), .b(new_n217), .c(new_n236), .out0(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n237), .b(new_n239), .c(new_n185), .d(new_n181), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  nand02aa1n10x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  nor042aa1n06x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nand02aa1d10x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n242), .c(new_n240), .d(new_n244), .o1(new_n248));
  nand42aa1n02x5               g153(.a(new_n240), .b(new_n244), .o1(new_n249));
  nona22aa1n02x4               g154(.a(new_n249), .b(new_n247), .c(new_n242), .out0(new_n250));
  nanp02aa1n02x5               g155(.a(new_n250), .b(new_n248), .o1(\s[24] ));
  nanp03aa1d12x5               g156(.a(new_n196), .b(new_n211), .c(new_n208), .o1(new_n252));
  tech160nm_fiao0012aa1n02p5x5 g157(.a(new_n245), .b(new_n242), .c(new_n246), .o(new_n253));
  nano23aa1d15x5               g158(.a(new_n242), .b(new_n245), .c(new_n246), .d(new_n243), .out0(new_n254));
  aoi012aa1n12x5               g159(.a(new_n253), .b(new_n254), .c(new_n235), .o1(new_n255));
  nand22aa1n09x5               g160(.a(new_n254), .b(new_n236), .o1(new_n256));
  aoai13aa1n12x5               g161(.a(new_n255), .b(new_n256), .c(new_n252), .d(new_n215), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  nano32aa1n03x7               g163(.a(new_n200), .b(new_n254), .c(new_n217), .d(new_n236), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n258), .b(new_n260), .c(new_n185), .d(new_n181), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  tech160nm_fixorc02aa1n03p5x5 g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  aoai13aa1n03x5               g170(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(new_n261), .b(new_n264), .o1(new_n267));
  nona22aa1n02x4               g172(.a(new_n267), .b(new_n265), .c(new_n263), .out0(new_n268));
  nanp02aa1n02x5               g173(.a(new_n268), .b(new_n266), .o1(\s[26] ));
  inv000aa1d42x5               g174(.a(\a[26] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\b[25] ), .o1(new_n271));
  oaoi03aa1n12x5               g176(.a(new_n270), .b(new_n271), .c(new_n263), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n264), .b(new_n265), .out0(new_n274));
  tech160nm_fiaoi012aa1n05x5   g179(.a(new_n273), .b(new_n257), .c(new_n274), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n256), .o1(new_n276));
  nanb03aa1n02x5               g181(.a(new_n219), .b(new_n274), .c(new_n276), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n275), .b(new_n277), .c(new_n185), .d(new_n181), .o1(new_n278));
  xorb03aa1n03x5               g183(.a(new_n278), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n280), .c(new_n278), .d(new_n281), .o1(new_n283));
  nand22aa1n03x5               g188(.a(new_n257), .b(new_n274), .o1(new_n284));
  nand02aa1n04x5               g189(.a(new_n284), .b(new_n272), .o1(new_n285));
  nano32aa1n03x7               g190(.a(new_n219), .b(new_n274), .c(new_n236), .d(new_n254), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n281), .b(new_n285), .c(new_n188), .d(new_n286), .o1(new_n287));
  nona22aa1n03x5               g192(.a(new_n287), .b(new_n282), .c(new_n280), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n283), .b(new_n288), .o1(\s[28] ));
  inv000aa1d42x5               g194(.a(\a[28] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(\b[27] ), .o1(new_n291));
  oaoi03aa1n09x5               g196(.a(new_n290), .b(new_n291), .c(new_n280), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  norb02aa1n02x5               g198(.a(new_n281), .b(new_n282), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n285), .c(new_n188), .d(new_n286), .o1(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  nona22aa1n03x5               g201(.a(new_n295), .b(new_n296), .c(new_n293), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n296), .b(new_n293), .c(new_n278), .d(new_n294), .o1(new_n298));
  nanp02aa1n03x5               g203(.a(new_n298), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  tech160nm_fioaoi03aa1n03p5x5 g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .o1(new_n301));
  norb03aa1n02x5               g206(.a(new_n281), .b(new_n296), .c(new_n282), .out0(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n301), .c(new_n278), .d(new_n302), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n302), .b(new_n285), .c(new_n188), .d(new_n286), .o1(new_n305));
  nona22aa1n03x5               g210(.a(new_n305), .b(new_n303), .c(new_n301), .out0(new_n306));
  nanp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  nanb02aa1n02x5               g212(.a(new_n303), .b(new_n301), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n308), .b(\b[29] ), .c(\a[30] ), .o1(new_n309));
  norb02aa1n02x5               g214(.a(new_n302), .b(new_n303), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n285), .c(new_n188), .d(new_n286), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nona22aa1n03x5               g217(.a(new_n311), .b(new_n312), .c(new_n309), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n312), .b(new_n309), .c(new_n278), .d(new_n310), .o1(new_n314));
  nanp02aa1n03x5               g219(.a(new_n314), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g224(.a(new_n104), .b(new_n123), .c(new_n124), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g226(.a(new_n106), .b(new_n125), .c(new_n123), .out0(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g228(.a(new_n100), .b(new_n322), .c(new_n107), .out0(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(new_n98), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


